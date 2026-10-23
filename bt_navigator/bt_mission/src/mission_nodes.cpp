#include "bt_mission/mission_nodes.hpp"

#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tf2/exceptions.h"

namespace bt_mission
{

namespace
{

// 같은 ROS 노드에서 나가는 상태 보고(mission/status)는 publisher 하나로 보낸다.
// 노드마다 publisher 를 따로 가지면 받는 쪽에 순서가 바뀌어 도착할 수 있다
// (예전 토픽 버전에서 "t1 started" 가 "t1 accepted" 보다 먼저 오는 것을 실제로 봤다).
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPublisher(
  const rclcpp::Node::SharedPtr & node)
{
  static std::map<const rclcpp::Node *, std::weak_ptr<rclcpp::Publisher<std_msgs::msg::String>>>
  publishers;
  auto & weak = publishers[node.get()];
  auto publisher = weak.lock();
  if (!publisher) {
    publisher = node->create_publisher<std_msgs::msg::String>(kStatusTopic, rclcpp::QoS(10));
    weak = publisher;
  }
  return publisher;
}

}  // namespace

DoWork::DoWork(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
{
}

BT::NodeStatus DoWork::onStart()
{
  task_ = name();
  getInput("task", task_);
  getInput("duration", duration_);
  start_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(), "[work] %s: start (%.1f s)", task_.c_str(), duration_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DoWork::onRunning()
{
  const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start_;
  if (elapsed.count() < duration_) {
    return BT::NodeStatus::RUNNING;
  }
  RCLCPP_INFO(node_->get_logger(), "[work] %s: done", task_.c_str());
  return BT::NodeStatus::SUCCESS;
}

void DoWork::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "[work] %s: interrupted", task_.c_str());
}

// ---------------------------------------------------------------------------------------------

ScanWhileMoving::ScanWhileMoving(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node")),
  tf_(config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"))
{
  // 나중에 켠 RViz 도 마지막 결과를 받을 수 있게 transient_local
  pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
    "mission/scan_points", rclcpp::QoS(1).transient_local());
}

BT::NodeStatus ScanWhileMoving::onStart()
{
  getInput("interval", interval_);
  getInput("global_frame", global_frame_);
  getInput("robot_frame", robot_frame_);
  scans_.poses.clear();
  scans_.header.frame_id = global_frame_;
  RCLCPP_INFO(node_->get_logger(), "[scan] start (every %.2f m)", interval_);
  return onRunning();
}

BT::NodeStatus ScanWhileMoving::onRunning()
{
  geometry_msgs::msg::TransformStamped robot;
  try {
    robot = tf_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return BT::NodeStatus::RUNNING;      // TF 가 아직 없으면 다음 tick 에 다시
  }
  const auto & t = robot.transform.translation;
  if (!scans_.poses.empty()) {
    const auto & last = scans_.poses.back().position;
    if (std::hypot(t.x - last.x, t.y - last.y) < interval_) {
      return BT::NodeStatus::RUNNING;
    }
  }

  // 실제로는 이 자리에서 센서 데이터를 저장하거나 서비스를 부른다
  geometry_msgs::msg::Pose pose;
  pose.position.x = t.x;
  pose.position.y = t.y;
  pose.orientation = robot.transform.rotation;
  scans_.poses.push_back(pose);
  scans_.header.stamp = node_->now();
  pub_->publish(scans_);
  RCLCPP_INFO(
    node_->get_logger(), "[scan] #%zu at (%.2f, %.2f)", scans_.poses.size(), t.x, t.y);
  return BT::NodeStatus::RUNNING;
}

void ScanWhileMoving::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[scan] stop: %zu scans", scans_.poses.size());
}

// ---------------------------------------------------------------------------------------------

ForEachWaypoint::ForEachWaypoint(const std::string & name, const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
{
  config.blackboard->get("tf_buffer", tf_);
  status_pub_ = statusPublisher(node_);
}

BT::NodeStatus ForEachWaypoint::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    poses_.clear();
    if (!getInput("poses", poses_) || poses_.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "%s: no poses to visit", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    index_ = 0;
    setOutput("reached", 0);
    setOutput("pose", poses_[0]);
  }
  setStatus(BT::NodeStatus::RUNNING);

  // 한 점에 도착하면 같은 tick 에 다음 점을 시작한다. 점 사이에 실행 중인 노드가 없는 tick 이 생기지 않는다
  while (true) {
    const auto child_status = child_node_->executeTick();
    if (child_status == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    haltChild();                         // 자식을 IDLE 로 돌려서 다음 점은 처음부터 시작하게 한다
    if (child_status == BT::NodeStatus::FAILURE) {
      return BT::NodeStatus::FAILURE;
    }
    ++index_;
    setOutput("reached", static_cast<int>(index_));
    reportArrival();
    if (index_ == poses_.size()) {
      return BT::NodeStatus::SUCCESS;
    }
    setOutput("pose", poses_[index_]);
  }
}

void ForEachWaypoint::halt()
{
  index_ = 0;
  BT::DecoratorNode::halt();
}

void ForEachWaypoint::reportArrival()
{
  std::string task_id, robot_frame;
  getInput("task_id", task_id);
  getInput("robot_frame", robot_frame);
  const auto & goal = poses_[index_ - 1];
  const std::string state =
    "arrived " + std::to_string(index_) + "/" + std::to_string(poses_.size());

  char where[96];
  std::snprintf(
    where, sizeof(where), " (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
  std::string detail = where;
  if (tf_) {
    try {
      const auto robot = tf_->lookupTransform(
        goal.header.frame_id, robot_frame, tf2::TimePointZero).transform.translation;
      std::snprintf(
        where, sizeof(where), ", robot %.3f m away",
        std::hypot(robot.x - goal.pose.position.x, robot.y - goal.pose.position.y));
      detail += where;
    } catch (const tf2::TransformException &) {
    }
  }
  const std::string prefix = task_id.empty() ? "" : task_id + " ";
  RCLCPP_INFO(
    node_->get_logger(), "[waypoint] %s%s%s", prefix.c_str(), state.c_str(), detail.c_str());
  std_msgs::msg::String status;
  status.data = prefix + state;
  status_pub_->publish(status);
}

// ---------------------------------------------------------------------------------------------

void registerNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<DoWork>("DoWork");
  factory.registerNodeType<ScanWhileMoving>("ScanWhileMoving");
  factory.registerNodeType<ForEachWaypoint>("ForEachWaypoint");
}

}  // namespace bt_mission
