// BT XML 을 읽어 트리를 만들고, 끝날 때까지 tick 한다.
//
//   ros2 run simple_line_mission mission_bt_runner                                  # mission.xml
//   ros2 run simple_line_mission mission_bt_runner --ros-args -p bt_xml:=task_loop.xml
//   ros2 run simple_line_mission mission_bt_runner --ros-args -p bt_xml:=/path/to/other.xml
//
// bt_xml 에 '/' 가 없으면 이 패키지의 share/.../behavior_trees/ 에서 찾는다.
//
// 순서 (1, 2 의 내용은 bt_common.cpp):
//   1. factory 에 BT 노드 종류를 등록한다 (Nav2 가 제공하는 노드 + 이 패키지의 노드)
//   2. blackboard 에 노드들이 같이 쓰는 값(ROS node, TF, timeout)을 넣는다
//   3. XML 로 트리를 만든다
//   4. 루트가 RUNNING 이 아닐 때까지 tick_hz 로 tick 한다

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "simple_line_mission/bt_common.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

std::atomic<bool> g_stop{false};

void onSignal(int)
{
  g_stop = true;
}

}  // namespace

int main(int argc, char ** argv)
{
  // Ctrl-C 는 직접 받는다. rclcpp 가 먼저 종료되면 실행 중인 Nav2 goal 을 취소할 수 없기 때문이다.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  auto node = std::make_shared<rclcpp::Node>("mission_bt");
  const std::string bt_xml = simple_line_mission::resolveTreePath(
    node->declare_parameter("bt_xml", std::string("mission.xml")));
  // bt_navigator 와 같은 100Hz (bt_loop_duration 10ms). Nav2 BT 노드는 tick 할 때마다 액션 메시지를
  // 조금씩만 처리하므로, 10Hz 로 낮추면 빠르게 오는 feedback 에 밀려 결과를 1초가량 늦게 받았다.
  const double tick_hz = node->declare_parameter("tick_hz", 100.0);
  const bool log_transitions = node->declare_parameter("log_transitions", true);

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer, node);   // 자체 스레드에서 TF 수신

  // 1. BT 노드 종류 등록
  BT::BehaviorTreeFactory factory;
  simple_line_mission::registerAllNodes(factory);

  // 2. blackboard
  auto blackboard = simple_line_mission::createBlackboard(node, tf_buffer, tick_hz);

  // 3. 트리 생성. Nav2 BT 노드는 생성될 때 액션 서버를 찾으므로, bt_navigator 가 뜬 뒤에 만든다
  if (!simple_line_mission::waitForActive(node, "bt_navigator", g_stop)) {
    rclcpp::shutdown();
    return 1;
  }
  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(bt_xml, blackboard);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "failed to create the tree from %s: %s", bt_xml.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }
  simple_line_mission::TransitionLogger logger(tree.rootNode(), node->get_logger(), log_transitions);
  RCLCPP_INFO(node->get_logger(), "running %s", bt_xml.c_str());

  // 4. tick 루프
  // tick 한 번 = 루트에서 시작해 지금 실행할 노드까지 내려가며 각 노드를 한 번씩 실행.
  // 오래 걸리는 노드(주행, 작업)는 RUNNING 을 돌려주고, 다음 tick 에서 이어서 확인한다.
  const auto start = std::chrono::steady_clock::now();
  rclcpp::WallRate rate(tick_hz);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING) {
    if (g_stop) {
      RCLCPP_WARN(node->get_logger(), "interrupted: halting the tree (cancels running goals)");
      tree.haltTree();
      break;
    }
    try {
      status = tree.tickRoot();
    } catch (const std::exception & e) {   // 포트 타입 불일치 등 노드가 던진 예외
      RCLCPP_ERROR(node->get_logger(), "BT error: %s", e.what());
      tree.haltTree();
      status = BT::NodeStatus::FAILURE;
      break;
    }
    rate.sleep();
  }

  const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
  RCLCPP_INFO(
    node->get_logger(), "mission %s in %.1f s",
    status == BT::NodeStatus::RUNNING ? "CANCELED" : BT::toStr(status).c_str(), elapsed.count());
  rclcpp::shutdown();
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
