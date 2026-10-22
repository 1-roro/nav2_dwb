// ExecuteTask 액션 서버 + BT 실행기. goal 하나(작업 하나)를 받을 때마다 task.xml 트리로 실행한다.
//
//   ros2 run simple_line_mission task_server
//   ros2 run simple_line_mission task_server --ros-args -p bt_xml:=/path/to/other_task.xml
//
// 작업 보내기: ros2 run simple_line_mission task_client.py
//            ros2 action send_goal --feedback /execute_task simple_line_mission_msgs/action/ExecuteTask "{task: ...}"

#include <atomic>
#include <csignal>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "simple_line_mission/bt_common.hpp"
#include "simple_line_mission/task_server.hpp"
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
  // Ctrl-C 는 직접 받는다. 실행 중인 작업을 멈추고(Nav2 goal 취소) 클라이언트에 결과를 보낸 뒤 끝낸다
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  auto node = std::make_shared<rclcpp::Node>("task_server");
  const std::string bt_xml = simple_line_mission::resolveTreePath(
    node->declare_parameter("bt_xml", std::string("task.xml")));
  const double tick_hz = node->declare_parameter("tick_hz", 100.0);
  const bool log_transitions = node->declare_parameter("log_transitions", true);

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer, node);

  BT::BehaviorTreeFactory factory;
  simple_line_mission::registerAllNodes(factory);
  auto blackboard = simple_line_mission::createBlackboard(node, tf_buffer, tick_hz);
  simple_line_mission::TaskServer::prepareBlackboard(blackboard);

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

  simple_line_mission::TaskServer server(node, tree, blackboard, log_transitions);
  RCLCPP_INFO(node->get_logger(), "ready: action /execute_task, tree %s", bt_xml.c_str());

  rclcpp::WallRate rate(tick_hz);
  while (!g_stop) {
    server.step();
    rate.sleep();
  }
  server.stop("task_server shutting down");
  rclcpp::shutdown();
  return 0;
}
