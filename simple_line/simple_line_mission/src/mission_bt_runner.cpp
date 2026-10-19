// mission.xml 을 읽어 BT 를 만들고, 끝날 때까지 tick 한다.
//
//   ros2 run simple_line_mission mission_bt_runner
//   ros2 run simple_line_mission mission_bt_runner --ros-args -p bt_xml:=/path/to/other.xml
//
// 순서:
//   1. factory 에 BT 노드 종류를 등록한다 (Nav2 가 제공하는 노드 + 이 패키지의 노드)
//   2. blackboard 에 노드들이 같이 쓰는 값(ROS node, TF, timeout)을 넣는다
//   3. XML 로 트리를 만든다
//   4. 루트가 RUNNING 이 아닐 때까지 tick_hz 로 tick 한다

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simple_line_mission/mission_nodes.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace
{

std::atomic<bool> g_stop{false};

void onSignal(int)
{
  g_stop = true;
}

// 상태가 바뀐 BT 노드를 한 줄씩 출력한다 (어느 노드가 언제 RUNNING / SUCCESS / FAILURE 가 되는지)
class TransitionLogger : public BT::StatusChangeLogger
{
public:
  TransitionLogger(BT::TreeNode * root, const rclcpp::Logger & logger)
  : BT::StatusChangeLogger(root), logger_(logger)
  {
  }

  void callback(
    BT::Duration, const BT::TreeNode & node,
    BT::NodeStatus prev, BT::NodeStatus status) override
  {
    // 끝난 노드가 IDLE 로 돌아가는 것은 생략하고, 실행 중에 멈춘 것(halt)만 보여 준다
    if (status == BT::NodeStatus::IDLE && prev != BT::NodeStatus::RUNNING) {
      return;
    }
    RCLCPP_INFO(
      logger_, "[bt] %-15s %-20s %s -> %s", node.name().c_str(),
      node.registrationName().c_str(), BT::toStr(prev).c_str(),
      status == BT::NodeStatus::IDLE ? "HALTED" : BT::toStr(status).c_str());
  }

  void flush() override {}

private:
  rclcpp::Logger logger_;
};

// lifecycle 노드가 active 가 될 때까지 기다린다. active 전에 보낸 goal 은 거절된다.
bool waitForActive(const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  auto client = node->create_client<lifecycle_msgs::srv::GetState>(name + "/get_state");
  while (!g_stop) {
    if (client->wait_for_service(1s)) {
      auto future = client->async_send_request(
        std::make_shared<lifecycle_msgs::srv::GetState::Request>());
      if (rclcpp::spin_until_future_complete(node, future, 1s) ==
        rclcpp::FutureReturnCode::SUCCESS &&
        future.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        return true;
      }
    }
    RCLCPP_INFO(node->get_logger(), "waiting for %s to become active ...", name.c_str());
    std::this_thread::sleep_for(1s);
  }
  return false;
}

}  // namespace

int main(int argc, char ** argv)
{
  // Ctrl-C 는 직접 받는다. rclcpp 가 먼저 종료되면 실행 중인 Nav2 goal 을 취소할 수 없기 때문이다.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  auto node = std::make_shared<rclcpp::Node>("mission_bt");
  const std::string bt_xml = node->declare_parameter(
    "bt_xml", ament_index_cpp::get_package_share_directory("simple_line_mission") +
    "/behavior_trees/mission.xml");
  // bt_navigator 와 같은 100Hz (bt_loop_duration 10ms). Nav2 BT 노드는 tick 할 때마다 액션 메시지를
  // 조금씩만 처리하므로, 10Hz 로 낮추면 빠르게 오는 feedback 에 밀려 결과를 1초가량 늦게 받았다.
  const double tick_hz = node->declare_parameter("tick_hz", 100.0);
  const bool log_transitions = node->declare_parameter("log_transitions", true);

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer, node);   // 자체 스레드에서 TF 수신

  // 1. BT 노드 종류 등록 ------------------------------------------------------------------------
  BT::BehaviorTreeFactory factory;
  // (a) Nav2 가 제공하는 BT 노드: 플러그인 라이브러리(.so)를 이름으로 불러온다.
  //     bt_navigator 의 plugin_lib_names 파라미터와 같은 방식이다.
  BT::SharedLibrary loader;
  for (const char * lib : {
      "nav2_navigate_to_pose_action_bt_node",       // <NavigateToPose>
      "nav2_navigate_through_poses_action_bt_node",  // <NavigateThroughPoses>
      "nav2_spin_action_bt_node",                    // <Spin>
      "nav2_wait_action_bt_node"})                   // <Wait>
  {
    factory.registerFromPlugin(loader.getOSName(lib));
  }
  // (b) 이 패키지에서 만든 BT 노드: 코드에서 바로 등록한다
  simple_line_mission::registerNodes(factory);

  // 2. blackboard ------------------------------------------------------------------------------
  // 노드들이 같이 쓰는 변수 저장소. Nav2 BT 노드는 생성될 때 아래 키들을 꼭 찾는다.
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer);
  blackboard->set<std::chrono::milliseconds>("server_timeout", 1000ms);
  blackboard->set<std::chrono::milliseconds>(
    "bt_loop_duration", std::chrono::milliseconds(static_cast<int>(1000.0 / tick_hz)));
  blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 5000ms);

  // 3. 트리 생성 --------------------------------------------------------------------------------
  // Nav2 BT 노드는 생성될 때 액션 서버를 찾으므로, bt_navigator 가 뜬 뒤에 만든다
  if (!waitForActive(node, "bt_navigator")) {
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
  std::unique_ptr<TransitionLogger> logger;
  if (log_transitions) {
    logger = std::make_unique<TransitionLogger>(tree.rootNode(), node->get_logger());
  }
  RCLCPP_INFO(node->get_logger(), "running %s", bt_xml.c_str());

  // 4. tick 루프 --------------------------------------------------------------------------------
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
