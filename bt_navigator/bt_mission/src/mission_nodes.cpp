#include "bt_mission/mission_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"  // PackageNotFoundError
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/exceptions.h"

namespace bt_mission
{

namespace
{

// 같은 ROS 노드에서 나가는 상태 보고(mission/status)는 publisher 하나로 보낸다.
// Log 와 WaitForTask 가 publisher 를 따로 가지면 받는 쪽에 순서가 바뀌어 도착할 수 있다
// ("t1 started" 가 "t1 accepted" 보다 먼저 오는 것을 실제로 봤다).
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

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::chrono::duration<double> seconds(double s)
{
  return std::chrono::duration<double>(s);
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

Log::Log(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
{
  status_pub_ = statusPublisher(node_);
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
  task_sub_ = node_->create_subscription<bt_mission_msgs::msg::Task>(
    topic, rclcpp::QoS(10),
    [this](bt_mission_msgs::msg::Task::ConstSharedPtr msg) {queue_.push_back(*msg);},
    options);
  status_pub_ = statusPublisher(node_);
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
    setOutput("obstacle", task.obstacle);
    report(
      task.id + " accepted: " + (task.action.empty() ? "go" : task.action) + ", " +
      std::to_string(task.waypoints.size()) + " poses" +
      (task.obstacle.empty() ? "" : ", obstacle " + task.obstacle));
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

ObstacleMode::ObstacleMode(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
{
}

BT::NodeStatus ObstacleMode::tick()
{
  using Task = bt_mission_msgs::msg::Task;
  std::string mode, avoid_bt;
  getInput("mode", mode);
  getInput("avoid_bt", avoid_bt);
  if (mode.empty() || mode == Task::OBSTACLE_STOP) {
    setOutput("stop_and_go", true);
    setOutput("nav_bt", std::string());                // bt_navigator 의 기본 트리
  } else if (mode == Task::OBSTACLE_AVOID) {
    if (avoid_bt.find('/') == std::string::npos) {
      try {
        avoid_bt = ament_index_cpp::get_package_share_directory("bt_mission") +
          "/behavior_trees/" + avoid_bt;
      } catch (const ament_index_cpp::PackageNotFoundError & e) {
        RCLCPP_ERROR(node_->get_logger(), "%s: %s", name().c_str(), e.what());
        return BT::NodeStatus::FAILURE;
      }
    }
    setOutput("stop_and_go", false);
    setOutput("nav_bt", avoid_bt);
  } else if (mode == Task::OBSTACLE_NONE) {
    setOutput("stop_and_go", false);
    setOutput("nav_bt", std::string());
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "%s: unknown obstacle mode '%s' (stop, avoid, none)",
      name().c_str(), mode.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------

StopAndGo::StopAndGo(const std::string & name, const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node")),
  scan_topic_("scan")
{
  config.blackboard->get("tf_buffer", tf_);
  status_pub_ = statusPublisher(node_);
  getInput("scan_topic", scan_topic_);
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
      // 받은 시각이 아니라 보낸 시각으로 오래됐는지 본다. 이 노드를 한동안 tick 하지 않았으면
      // 큐에 남아 있던 scan 을 지금 처리하게 되는데, 그 scan 이 새것처럼 보이면 안 되기 때문이다
      const auto sent = info.get_rmw_message_info().source_timestamp;
      scan_time_ = sent > 0 ?
      std::chrono::system_clock::time_point(std::chrono::nanoseconds(sent)) :
      std::chrono::system_clock::now();
      scan_ = msg;
    },
    options);
}

BT::NodeStatus StopAndGo::tick()
{
  bool enabled = true;
  getInput("enabled", enabled);
  if (!enabled) {                        // avoid, none: 장애물은 보지 않는다
    setStatus(BT::NodeStatus::RUNNING);
    return child_node_->executeTick();
  }
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    RCLCPP_ERROR(node_->get_logger(), "%s: no goal", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  setStatus(BT::NodeStatus::RUNNING);
  executor_.spin_some();                 // 도착한 scan 을 받는다
  const auto check = checkPath(goal);
  const auto now = std::chrono::steady_clock::now();

  if (state_ != State::kWaiting) {
    if (!check.blocked) {
      if (state_ == State::kIdle) {
        setOutput("blocked", false);
      }
      return startChild();
    }
    // 길이 막혔다. 가고 있었으면 자식을 멈춘다 (NavigateToPose 의 halt = Nav2 goal 취소 → 로봇 정지)
    haltChild();
    state_ = State::kWaiting;
    blocked_since_ = now;
    clear_ = false;
    setOutput("blocked", true);
    report("blocked: " + check.reason, true);
    return BT::NodeStatus::RUNNING;
  }

  // 멈춰서 기다리는 중
  char text[64];
  if (check.blocked) {
    clear_ = false;
    double max_wait = 0.0;
    getInput("max_wait", max_wait);
    if (max_wait > 0.0 && now - blocked_since_ > seconds(max_wait)) {
      std::snprintf(text, sizeof(text), "gave up: blocked for %.1f s", max_wait);
      report(text, true);
      state_ = State::kIdle;
      return BT::NodeStatus::FAILURE;    // blocked 는 true 로 둔다: 서버가 실패 이유로 쓴다
    }
    return BT::NodeStatus::RUNNING;
  }
  if (!clear_) {
    clear_ = true;
    clear_since_ = now;
  }
  double resume_delay = 1.0;
  getInput("resume_delay", resume_delay);
  if (now - clear_since_ < seconds(resume_delay)) {
    return BT::NodeStatus::RUNNING;      // 잠깐 비었다가 다시 막히는 경우를 걸러 낸다
  }
  std::snprintf(
    text, sizeof(text), "resumed after %.1f s",
    std::chrono::duration<double>(now - blocked_since_).count());
  report(text, false);
  setOutput("blocked", false);
  return startChild();                   // 같은 goal 을 다시 보낸다. 경로는 지금 위치에서 다시 계산된다
}

BT::NodeStatus StopAndGo::startChild()
{
  state_ = State::kGoing;
  const auto status = child_node_->executeTick();
  if (status != BT::NodeStatus::RUNNING) {
    state_ = State::kIdle;
  }
  return status;
}

void StopAndGo::halt()
{
  state_ = State::kIdle;
  clear_ = false;
  BT::DecoratorNode::halt();
}

StopAndGo::PathCheck StopAndGo::checkPath(const geometry_msgs::msg::PoseStamped & goal)
{
  double scan_timeout = 1.0;
  getInput("scan_timeout", scan_timeout);
  if (!scan_ || std::chrono::system_clock::now() - scan_time_ > seconds(scan_timeout)) {
    return {true, "no scan on " + scan_topic_};
  }
  if (!tf_) {
    return {true, "no tf_buffer on the blackboard"};
  }
  std::string robot_frame = "base_link";
  getInput("robot_frame", robot_frame);
  geometry_msgs::msg::Transform robot, sensor;
  try {
    robot = tf_->lookupTransform(goal.header.frame_id, robot_frame, tf2::TimePointZero).transform;
    sensor = tf_->lookupTransform(
      goal.header.frame_id, scan_->header.frame_id, tf2::TimePointZero).transform;
  } catch (const tf2::TransformException & e) {
    return {true, std::string("no TF (") + e.what() + ")"};
  }

  // 길: 로봇에서 goal 쪽으로 난 직선. SimpleLinePlanner 가 긋는 경로와 같다
  const double rx = robot.translation.x;
  const double ry = robot.translation.y;
  const double dx = goal.pose.position.x - rx;
  const double dy = goal.pose.position.y - ry;
  const double dist = std::hypot(dx, dy);
  if (dist < 1e-3) {
    return {};                           // goal 위에 있다 (방향만 맞추면 된다)
  }
  const double ux = dx / dist;
  const double uy = dy / dist;
  double stop_distance = 0.6;
  double half_width = 0.25;
  getInput("stop_distance", stop_distance);
  getInput("half_width", half_width);
  const double reach = std::min(stop_distance, dist);   // goal 너머의 물체는 보지 않는다

  // scan 의 점을 goal 의 frame 으로 옮겨서, 길 방향(along)과 옆 방향(side) 거리로 나눈다
  const double c = std::cos(yawOf(sensor.rotation));
  const double s = std::sin(yawOf(sensor.rotation));
  double nearest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < scan_->ranges.size(); ++i) {
    const float r = scan_->ranges[i];
    if (!std::isfinite(r) || r < scan_->range_min || r > scan_->range_max) {
      continue;
    }
    const double a = scan_->angle_min + static_cast<double>(i) * scan_->angle_increment;
    const double lx = r * std::cos(a);
    const double ly = r * std::sin(a);
    const double px = sensor.translation.x + c * lx - s * ly - rx;
    const double py = sensor.translation.y + s * lx + c * ly - ry;
    const double along = px * ux + py * uy;
    const double side = std::abs(py * ux - px * uy);
    if (along > 0.0 && along <= reach && side <= half_width) {
      nearest = std::min(nearest, along);
    }
  }
  if (std::isinf(nearest)) {
    return {};
  }
  char text[48];
  std::snprintf(text, sizeof(text), "obstacle %.2f m ahead", nearest);
  return {true, text};
}

void StopAndGo::report(const std::string & text, bool warn)
{
  std::string task_id;
  getInput("task_id", task_id);
  std_msgs::msg::String status;
  status.data = (task_id.empty() ? "" : task_id + " ") + text;
  if (warn) {
    RCLCPP_WARN(node_->get_logger(), "[stop&go] %s", status.data.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "[stop&go] %s", status.data.c_str());
  }
  status_pub_->publish(status);
}

// ---------------------------------------------------------------------------------------------

void registerNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<DoWork>("DoWork");
  factory.registerNodeType<ScanWhileMoving>("ScanWhileMoving");
  factory.registerNodeType<Log>("Log");
  factory.registerNodeType<WaitForTask>("WaitForTask");
  factory.registerNodeType<ForEachWaypoint>("ForEachWaypoint");
  factory.registerNodeType<ObstacleMode>("ObstacleMode");
  factory.registerNodeType<StopAndGo>("StopAndGo");
}

}  // namespace bt_mission
