#include "bt_mission/dock_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace bt_mission
{

namespace
{

double secondsSince(std::chrono::steady_clock::time_point t)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}

geometry_msgs::msg::Twist toTwist(const dock::Cmd & cmd)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = cmd.vx;
  twist.linear.y = cmd.vy;
  twist.angular.z = cmd.wz;
  return twist;
}

constexpr double kControlPeriod = 0.05;    // 명령을 내는 주기 [s] (velocity_smoother, controller 와 같은 20 Hz)

}  // namespace

// ---------------------------------------------------------------------------------------------

DockRos::DockRos(const BT::NodeConfiguration & config)
: node(config.blackboard->get<rclcpp::Node::SharedPtr>("node")),
  tf(config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer")),
  group(node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false))
{
  executor.add_callback_group(group, node->get_node_base_interface());
}

std::optional<dock::Pose2> DockRos::robotPose(const std::string & global, const std::string & robot) const
{
  try {
    const auto t = tf->lookupTransform(global, robot, tf2::TimePointZero);
    return dock::Pose2{t.transform.translation.x, t.transform.translation.y,
      tf2::getYaw(t.transform.rotation)};
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

geometry_msgs::msg::PoseStamped toPoseStamped(const dock::Pose2 & pose, const std::string & frame)
{
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = frame;
  out.pose.position.x = pose.x;
  out.pose.position.y = pose.y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.yaw);
  out.pose.orientation = tf2::toMsg(q);
  return out;
}

dock::Pose2 toPose2(const geometry_msgs::msg::PoseStamped & pose)
{
  return {pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation)};
}

// ---------------------------------------------------------------------------------------------

BackOff::BackOff(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), ros_(config)
{
}

BT::NodeStatus BackOff::onStart()
{
  double distance = 0.4;
  std::string cmd_topic = "cmd_vel_nav", scan_topic = "scan";
  getInput("distance", distance);
  getInput("speed", params_.max_lin);
  getInput("timeout", timeout_);
  getInput("scan_topic", scan_topic);
  getInput("robot_rear", rear_);
  getInput("robot_half_width", half_width_);
  getInput("clearance", clearance_);
  getInput("fail_when_blocked", fail_when_blocked_);
  getInput("global_frame", global_frame_);
  getInput("robot_frame", robot_frame_);
  getInput("cmd_vel_topic", cmd_topic);
  if (!cmd_pub_) {
    cmd_pub_ = ros_.node->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, rclcpp::QoS(10));
  }
  scan_.reset();
  scan_sub_.reset();
  if (!scan_topic.empty()) {
    rclcpp::SubscriptionOptions options;
    options.callback_group = ros_.group;
    scan_sub_ = ros_.node->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_ = msg;
        scan_time_ = std::chrono::steady_clock::now();
      }, options);
  }
  const auto robot = ros_.robotPose(global_frame_, robot_frame_);
  if (!robot) {
    RCLCPP_ERROR(ros_.node->get_logger(), "[dock] %s: no robot pose", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  goal_ = {robot->x - distance * std::cos(robot->yaw), robot->y - distance * std::sin(robot->yaw), robot->yaw};
  params_.xy_tolerance = 0.01;
  params_.yaw_tolerance = 2.0 * M_PI / 180.0;
  last_cmd_ = dock::Cmd{};
  start_ = last_cmd_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(ros_.node->get_logger(), "[dock] %s: back off %.2f m", name().c_str(), distance);
  return BT::NodeStatus::RUNNING;
}

void BackOff::publish(const dock::Cmd & cmd)
{
  cmd_pub_->publish(toTwist(cmd));
  last_cmd_ = cmd;
}

BT::NodeStatus BackOff::stop(BT::NodeStatus status, const char * why)
{
  publish(dock::Cmd{});
  scan_sub_.reset();
  if (status == BT::NodeStatus::SUCCESS && std::string(why) == "done") {
    RCLCPP_INFO(ros_.node->get_logger(), "[dock] %s: %s in %.1f s", name().c_str(), why, secondsSince(start_));
  } else {
    RCLCPP_WARN(ros_.node->get_logger(), "[dock] %s: %s after %.1f s", name().c_str(), why, secondsSince(start_));
  }
  return status;
}

std::optional<std::vector<dock::Point2>> scanPoints(
  const tf2_ros::Buffer & tf, const sensor_msgs::msg::LaserScan & scan, const std::string & robot_frame)
{
  geometry_msgs::msg::TransformStamped mount;     // 레이저가 로봇에 달린 자리 (움직이지 않는다)
  try {
    mount = tf.lookupTransform(robot_frame, scan.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
  const double mx = mount.transform.translation.x, my = mount.transform.translation.y;
  const double myaw = tf2::getYaw(mount.transform.rotation);
  std::vector<dock::Point2> points;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
      continue;
    }
    const double a = myaw + scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    points.push_back({mx + r * std::cos(a), my + r * std::sin(a)});
  }
  return points;
}

int BackOff::pointsBehind(const sensor_msgs::msg::LaserScan & scan, double distance)
{
  const auto points = scanPoints(*ros_.tf, scan, robot_frame_);
  if (!points) {
    return -1;
  }
  return dock::pointsBehind(*points, rear_, half_width_, distance, clearance_);
}

BT::NodeStatus BackOff::onRunning()
{
  if (secondsSince(start_) > timeout_) {
    return stop(BT::NodeStatus::FAILURE, "timed out");
  }
  ros_.spin();
  const double dt = secondsSince(last_cmd_time_);
  if (dt < kControlPeriod) {
    return BT::NodeStatus::RUNNING;
  }
  last_cmd_time_ = std::chrono::steady_clock::now();
  const auto robot = ros_.robotPose(global_frame_, robot_frame_);
  if (!robot) {
    publish(dock::Cmd{});
    return BT::NodeStatus::RUNNING;
  }
  const dock::Pose2 error = dock::errorInRobotFrame(*robot, goal_);
  if (dock::withinTolerance(error, params_)) {
    return stop(BT::NodeStatus::SUCCESS, "done");
  }
  if (scan_sub_) {
    // 뒤를 본 뒤에만 움직인다. 레이저가 끊기면 선다
    if (!scan_ || secondsSince(scan_time_) > 1.0) {
      if (secondsSince(start_) > 2.0) {
        return stop(BT::NodeStatus::FAILURE, "no laser scan to check behind");
      }
      publish(dock::Cmd{});
      return BT::NodeStatus::RUNNING;
    }
    const int behind = pointsBehind(*scan_, std::max(0.0, -error.x));
    if (behind < 0) {
      return stop(BT::NodeStatus::FAILURE, "no transform for the laser");
    }
    if (behind > 3) {
      return stop(
        fail_when_blocked_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS, "something is behind the robot, stopped");
    }
  }
  publish(dock::holonomicStep(error, last_cmd_, params_, std::min(dt, 0.2)));
  return BT::NodeStatus::RUNNING;
}

void BackOff::onHalted()
{
  publish(dock::Cmd{});
  scan_sub_.reset();
}

// ---------------------------------------------------------------------------------------------

MuteCollisionMonitor::MuteCollisionMonitor(const std::string & name, const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config), ros_(config)
{
}

bool MuteCollisionMonitor::setEnabled(bool enabled, std::chrono::milliseconds wait)
{
  auto future = client_->set_parameters({rclcpp::Parameter(polygon_ + ".enabled", enabled)});
  if (wait.count() == 0) {
    pending_ = future;
    return true;
  }
  if (ros_.executor.spin_until_future_complete(future, wait) != rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }
  const auto results = future.get();
  return !results.empty() && results.front().successful;
}

BT::NodeStatus MuteCollisionMonitor::tick()
{
  if (state_ == State::Idle) {
    monitor_ = "collision_monitor";
    polygon_ = "Footprint";
    getInput("monitor", monitor_);
    getInput("polygon", polygon_);
    if (!client_) {
      client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        ros_.node, monitor_, rmw_qos_profile_parameters, ros_.group);
    }
    start_ = std::chrono::steady_clock::now();
    state_ = State::Muting;
    pending_ = {};
  }
  if (state_ == State::Muting) {
    double timeout = 2.0;
    getInput("timeout", timeout);
    if (secondsSince(start_) > timeout) {
      RCLCPP_ERROR(
        ros_.node->get_logger(), "[dock] could not switch off %s/%s", monitor_.c_str(), polygon_.c_str());
      state_ = State::Idle;
      return BT::NodeStatus::FAILURE;
    }
    if (!pending_.valid()) {
      if (!client_->service_is_ready()) {
        ros_.spin();
        return BT::NodeStatus::RUNNING;
      }
      setEnabled(false, 0ms);
    }
    ros_.spin();
    if (pending_.wait_for(0s) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    const auto results = pending_.get();
    if (results.empty() || !results.front().successful) {
      RCLCPP_ERROR(
        ros_.node->get_logger(), "[dock] %s refused to switch off %s", monitor_.c_str(), polygon_.c_str());
      state_ = State::Idle;
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(ros_.node->get_logger(), "[dock] %s: %s off", monitor_.c_str(), polygon_.c_str());
    state_ = State::Muted;
  }
  const BT::NodeStatus status = child_node_->executeTick();
  if (status == BT::NodeStatus::RUNNING) {
    return status;
  }
  state_ = State::Idle;
  if (!restore()) {
    return BT::NodeStatus::FAILURE;      // 자식이 성공했어도: 안전 구역이 꺼진 채로 다음 일을 하지 않는다
  }
  return status;
}

bool MuteCollisionMonitor::restore()
{
  // 응답이 늦을 수 있다 (Gazebo 에서 500 ms 를 넘긴 적이 있다). 몇 번 다시 보내며 기다린다
  for (int attempt = 1; attempt <= 3; ++attempt) {
    if (setEnabled(true, 1000ms)) {
      RCLCPP_INFO(ros_.node->get_logger(), "[dock] %s: %s on again", monitor_.c_str(), polygon_.c_str());
      return true;
    }
    RCLCPP_WARN(
      ros_.node->get_logger(), "[dock] %s: no answer to switching %s back on (%d/3)", monitor_.c_str(),
      polygon_.c_str(), attempt);
  }
  RCLCPP_ERROR(
    ros_.node->get_logger(), "[dock] could not switch %s/%s back on: the safety field may still be off",
    monitor_.c_str(), polygon_.c_str());
  return false;
}

void MuteCollisionMonitor::halt()
{
  if (state_ != State::Idle) {
    RCLCPP_WARN(ros_.node->get_logger(), "[dock] halted while %s was off", polygon_.c_str());
    state_ = State::Idle;
    restore();
  }
  BT::DecoratorNode::halt();
}

// ---------------------------------------------------------------------------------------------

CheckCharging::CheckCharging(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), ros_(config)
{
}

BT::NodeStatus CheckCharging::onStart()
{
  std::string topic;
  getInput("topic", topic);
  getInput("timeout", timeout_);
  if (topic.empty()) {
    RCLCPP_INFO(ros_.node->get_logger(), "[dock] no charging signal configured, not checked");
    return BT::NodeStatus::SUCCESS;
  }
  charging_ = false;
  rclcpp::SubscriptionOptions options;
  options.callback_group = ros_.group;
  sub_ = ros_.node->create_subscription<std_msgs::msg::Bool>(
    topic, rclcpp::QoS(10), [this](std_msgs::msg::Bool::SharedPtr msg) {charging_ = charging_ || msg->data;},
    options);
  start_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckCharging::onRunning()
{
  ros_.spin();
  if (charging_) {
    RCLCPP_INFO(ros_.node->get_logger(), "[dock] charging");
    sub_.reset();
    return BT::NodeStatus::SUCCESS;
  }
  if (secondsSince(start_) > timeout_) {
    RCLCPP_WARN(ros_.node->get_logger(), "[dock] no charging signal in %.1f s", timeout_);
    sub_.reset();
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void CheckCharging::onHalted()
{
  sub_.reset();
}

// ---------------------------------------------------------------------------------------------

void registerDockNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<BackOff>("BackOff");
  factory.registerNodeType<MuteCollisionMonitor>("MuteCollisionMonitor");
  factory.registerNodeType<CheckCharging>("CheckCharging");
}

}  // namespace bt_mission
