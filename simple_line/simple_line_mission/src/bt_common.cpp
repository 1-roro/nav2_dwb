#include "simple_line_mission/bt_common.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "simple_line_mission/mission_nodes.hpp"

using namespace std::chrono_literals;

namespace simple_line_mission
{

void registerAllNodes(BT::BehaviorTreeFactory & factory)
{
  BT::SharedLibrary loader;
  for (const char * lib : {
      "nav2_navigate_to_pose_action_bt_node",       // <NavigateToPose>
      "nav2_navigate_through_poses_action_bt_node",  // <NavigateThroughPoses>
      "nav2_spin_action_bt_node",                    // <Spin>
      "nav2_wait_action_bt_node"})                   // <Wait>
  {
    factory.registerFromPlugin(loader.getOSName(lib));
  }
  registerNodes(factory);
}

BT::Blackboard::Ptr createBlackboard(
  const rclcpp::Node::SharedPtr & node, const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  double tick_hz)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer);
  blackboard->set<std::chrono::milliseconds>("server_timeout", 1000ms);
  blackboard->set<std::chrono::milliseconds>(
    "bt_loop_duration", std::chrono::milliseconds(static_cast<int>(1000.0 / tick_hz)));
  blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 5000ms);
  return blackboard;
}

std::string resolveTreePath(const std::string & name)
{
  if (name.find('/') != std::string::npos) {
    return name;
  }
  return ament_index_cpp::get_package_share_directory("simple_line_mission") +
         "/behavior_trees/" + name;
}

bool waitForActive(
  const rclcpp::Node::SharedPtr & node, const std::string & name, const std::atomic<bool> & stop)
{
  auto client = node->create_client<lifecycle_msgs::srv::GetState>(name + "/get_state");
  while (!stop) {
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

// ---------------------------------------------------------------------------------------------

TransitionLogger::TransitionLogger(
  BT::TreeNode * root, const rclcpp::Logger & logger, bool print)
: BT::StatusChangeLogger(root), logger_(logger), print_(print)
{
}

void TransitionLogger::callback(
  BT::Duration, const BT::TreeNode & node, BT::NodeStatus prev, BT::NodeStatus status)
{
  if (node.type() == BT::NodeType::ACTION) {
    auto it = std::find(running_.begin(), running_.end(), node.name());
    if (status == BT::NodeStatus::RUNNING && it == running_.end()) {
      running_.push_back(node.name());
    } else if (status != BT::NodeStatus::RUNNING && it != running_.end()) {
      running_.erase(it);
    }
    if (status == BT::NodeStatus::FAILURE) {
      last_failed_ = node.name();
    }
  }
  // 끝난 노드가 IDLE 로 돌아가는 것은 생략하고, 실행 중에 멈춘 것(halt)만 보여 준다
  if (!print_ || (status == BT::NodeStatus::IDLE && prev != BT::NodeStatus::RUNNING)) {
    return;
  }
  RCLCPP_INFO(
    logger_, "[bt] %-15s %-23s %s -> %s", node.name().c_str(),
    node.registrationName().c_str(), BT::toStr(prev).c_str(),
    status == BT::NodeStatus::IDLE ? "HALTED" : BT::toStr(status).c_str());
}

std::string TransitionLogger::running() const
{
  std::string out;
  for (const auto & name : running_) {
    out += (out.empty() ? "" : ", ") + name;
  }
  return out;
}

void TransitionLogger::clear()
{
  running_.clear();
  last_failed_.clear();
}

}  // namespace simple_line_mission
