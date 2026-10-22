#include "simple_line_mission/mission_nodes.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "tf2/exceptions.h"

namespace simple_line_mission
{

namespace
{

std::string trim(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\n\r");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\n\r");
  return s.substr(begin, end - begin + 1);
}

std::vector<std::string> split(const std::string & s, char delim)
{
  std::vector<std::string> out;
  std::string::size_type start = 0;
  while (true) {
    const auto pos = s.find(delim, start);
    out.push_back(s.substr(start, pos == std::string::npos ? std::string::npos : pos - start));
    if (pos == std::string::npos) {
      return out;
    }
    start = pos + 1;
  }
}

double toDouble(const std::string & text)
{
  const std::string s = trim(text);
  std::size_t used = 0;
  double value = 0.0;
  try {
    value = std::stod(s, &used);
  } catch (const std::exception &) {
    used = 0;
  }
  if (s.empty() || used != s.size()) {
    throw std::invalid_argument("'" + text + "' is not a number");
  }
  return value;
}

}  // namespace

std::vector<Pose2D> parsePoses(const std::string & text)
{
  std::vector<Pose2D> poses;
  std::vector<bool> has_yaw;
  for (const auto & item : split(text, ';')) {
    if (trim(item).empty()) {
      continue;
    }
    const auto fields = split(item, ',');
    if (fields.size() != 2 && fields.size() != 3) {
      throw std::invalid_argument("'" + trim(item) + "' is not 'x,y' or 'x,y,yaw_deg'");
    }
    Pose2D p;
    p.x = toDouble(fields[0]);
    p.y = toDouble(fields[1]);
    if (fields.size() == 3) {
      p.yaw = toDouble(fields[2]) * M_PI / 180.0;
    }
    poses.push_back(p);
    has_yaw.push_back(fields.size() == 3);
  }
  if (poses.empty()) {
    throw std::invalid_argument("no pose in '" + text + "'");
  }

  for (std::size_t i = 0; i < poses.size(); ++i) {
    if (has_yaw[i]) {
      continue;
    }
    if (i + 1 < poses.size()) {          // 중간 점: 다음 점 쪽
      poses[i].yaw = std::atan2(poses[i + 1].y - poses[i].y, poses[i + 1].x - poses[i].x);
    } else if (i > 0) {                  // 마지막 점: 이전 점에서 온 방향
      poses[i].yaw = std::atan2(poses[i].y - poses[i - 1].y, poses[i].x - poses[i - 1].x);
    } else {
      throw std::invalid_argument("a single pose needs a yaw: '" + trim(text) + "'");
    }
  }
  return poses;
}

geometry_msgs::msg::PoseStamped toPoseStamped(
  const Pose2D & pose, const std::string & frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = frame;
  out.header.stamp = stamp;
  out.pose.position.x = pose.x;
  out.pose.position.y = pose.y;
  out.pose.orientation.z = std::sin(pose.yaw / 2.0);
  out.pose.orientation.w = std::cos(pose.yaw / 2.0);
  return out;
}

// ---------------------------------------------------------------------------------------------

MakePose::MakePose(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
{
}

BT::NodeStatus MakePose::tick()
{
  std::string text, frame;
  getInput("pose", text);
  getInput("frame", frame);
  try {
    const auto poses = parsePoses(text);
    if (poses.size() != 1) {
      throw std::invalid_argument("expected one pose, got " + std::to_string(poses.size()));
    }
    setOutput("output", toPoseStamped(poses.front(), frame, node_->now()));
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(node_->get_logger(), "%s: %s", name().c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------

MakePoses::MakePoses(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
{
}

BT::NodeStatus MakePoses::tick()
{
  std::string text, frame;
  getInput("poses", text);
  getInput("frame", frame);
  std::vector<geometry_msgs::msg::PoseStamped> out;
  try {
    const auto stamp = node_->now();
    for (const auto & pose : parsePoses(text)) {
      out.push_back(toPoseStamped(pose, frame, stamp));
    }
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(node_->get_logger(), "%s: %s", name().c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }
  setOutput("output", out);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------

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

Log::Log(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
{
  status_pub_ = node_->create_publisher<std_msgs::msg::String>(kStatusTopic, rclcpp::QoS(10));
}

BT::NodeStatus Log::tick()
{
  std::string message, task_id;
  getInput("message", message);
  getInput("task_id", task_id);
  std_msgs::msg::String status;
  status.data = task_id.empty() ? message : task_id + " " + message;
  RCLCPP_INFO(node_->get_logger(), "[log] %s", status.data.c_str());
  status_pub_->publish(status);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------

WaitForTask::WaitForTask(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
{
  std::string topic = "mission/task";
  getInput("topic", topic);
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  task_sub_ = node_->create_subscription<simple_line_mission_msgs::msg::Task>(
    topic, rclcpp::QoS(10),
    [this](simple_line_mission_msgs::msg::Task::ConstSharedPtr msg) {queue_.push_back(*msg);},
    options);
  status_pub_ = node_->create_publisher<std_msgs::msg::String>(kStatusTopic, rclcpp::QoS(10));
}

BT::NodeStatus WaitForTask::onStart()
{
  const auto status = takeTask();
  if (status == BT::NodeStatus::RUNNING) {
    report("waiting for a task");
  }
  return status;
}

BT::NodeStatus WaitForTask::onRunning()
{
  return takeTask();
}

BT::NodeStatus WaitForTask::takeTask()
{
  executor_.spin_some();                 // 도착한 Task 메시지를 queue_ 로 옮긴다
  while (!queue_.empty()) {
    auto task = queue_.front();
    queue_.pop_front();
    ++count_;
    if (task.id.empty()) {
      task.id = "task" + std::to_string(count_);
    }
    if (task.waypoints.empty()) {
      report(task.id + " rejected: no waypoints");
      continue;
    }
    for (auto & pose : task.waypoints) {
      if (pose.header.frame_id.empty()) {
        pose.header.frame_id = "map";
      }
    }
    setOutput("waypoints", task.waypoints);
    setOutput("task_id", task.id);
    setOutput("action", task.action);
    setOutput("duration", task.duration);
    report(
      task.id + " accepted: " + (task.action.empty() ? "go" : task.action) + ", " +
      std::to_string(task.waypoints.size()) + " poses");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForTask::report(const std::string & text)
{
  RCLCPP_INFO(node_->get_logger(), "[task] %s", text.c_str());
  std_msgs::msg::String status;
  status.data = text;
  status_pub_->publish(status);
}

// ---------------------------------------------------------------------------------------------

void registerNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<MakePose>("MakePose");
  factory.registerNodeType<MakePoses>("MakePoses");
  factory.registerNodeType<DoWork>("DoWork");
  factory.registerNodeType<ScanWhileMoving>("ScanWhileMoving");
  factory.registerNodeType<Log>("Log");
  factory.registerNodeType<WaitForTask>("WaitForTask");
}

}  // namespace simple_line_mission
