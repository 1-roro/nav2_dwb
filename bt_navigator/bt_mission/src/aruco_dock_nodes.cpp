#include "bt_mission/aruco_dock_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

namespace bt_mission
{

namespace
{

namespace ad = aruco_dock;

constexpr double kControlPeriod = 0.05;   // [s]
constexpr double kGate = 16.3;            // Mahalanobis 거리 제곱, 자유도 3 의 99.9 %
constexpr int kReinitAfter = 8;           // 연달아 이만큼 버리면 다시 잡는다

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

double deg(double rad) {return rad * 180.0 / M_PI;}

bool fileExists(const std::string & path)
{
  std::ifstream f(path);
  return f.good();
}

std::string describe(const ad::DockErrors & e)
{
  char text[120];
  std::snprintf(text, sizeof(text), "along %+.1f mm, lateral %+.1f mm, yaw %+.2f deg", e.along * 1000.0,
    e.lat * 1000.0, deg(e.yaw));
  return text;
}

std::string sigmas(const ad::Mat3 & cov)
{
  char text[100];
  std::snprintf(text, sizeof(text), "sigma x %.1f y %.1f mm, yaw %.2f deg", std::sqrt(cov[0]) * 1000.0,
    std::sqrt(cov[4]) * 1000.0, deg(std::sqrt(cov[8])));
  return text;
}

}  // namespace

ad::Hypotheses arucoMeasurements(
  const tf2_ros::Buffer & tf, const bt_mission_msgs::msg::ArucoDetection & d, const std::string & global)
{
  geometry_msgs::msg::TransformStamped stamped;
  try {
    stamped = tf.lookupTransform(global, d.header.frame_id, d.header.stamp, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException &) {
    return {};                                    // 영상을 찍은 시각의 로봇 자리를 모른다
  }
  const dock::Pose2 base{stamped.transform.translation.x, stamped.transform.translation.y,
    tf2::getYaw(stamped.transform.rotation)};
  const double c = std::cos(base.yaw), s = std::sin(base.yaw);
  ad::Hypotheses out;
  for (const auto & h : d.hypotheses) {
    ad::Measurement m;
    m.pose = ad::compose(base, {h.x, h.y, h.yaw});
    // 공분산: base 의 (x, y) 를 global 로 돌린다 (yaw 는 그대로). C = R Σ Rᵀ, R = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    const ad::Mat3 r{c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0};
    const ad::Mat3 rt{c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0};
    ad::Mat3 cov;
    std::copy(h.covariance.begin(), h.covariance.end(), cov.begin());
    m.cov = ad::mul(ad::mul(r, cov), rt);
    m.cov[0] += 1e-10;
    m.cov[4] += 1e-10;
    m.cov[8] += 1e-10;
    out.push_back(m);
  }
  return out;
}

// ---------------------------------------------------------------------------------------------

void ScanWatch::start(DockRos & ros, const std::string & topic)
{
  scan_.reset();
  sub_.reset();
  if (topic.empty()) {
    return;
  }
  rclcpp::SubscriptionOptions options;
  options.callback_group = ros.group;
  sub_ = ros.node->create_subscription<sensor_msgs::msg::LaserScan>(
    topic, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
      scan_ = msg;
      time_ = std::chrono::steady_clock::now();
    }, options);
}

std::optional<std::vector<dock::Point2>> ScanWatch::points(
  const tf2_ros::Buffer & tf, const std::string & robot_frame, std::string * why) const
{
  if (!sub_) {
    *why = "no laser to check around the robot (ros.scan_topic is empty)";
    return std::nullopt;
  }
  if (!scan_ || secondsSince(time_) > 1.0) {
    *why = "no recent laser scan";
    return std::nullopt;
  }
  auto points = scanPoints(tf, *scan_, robot_frame);
  if (!points) {
    *why = "no transform for the laser";
  }
  return points;
}

// ---------------------------------------------------------------------------------------------

DetectArucoDock::DetectArucoDock(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), ros_(config)
{
}

std::optional<dock::Pose2> DetectArucoDock::taughtTarget(const std::string & file, std::string * why)
{
  // aruco_teach.py 가 남긴 것: 제대로 붙은 로봇에서 aruco_detector.py 가 본 dock 의 자세 (base 에서, x y yaw).
  // 설 곳 = 그 역 (dock 에서 본 로봇). 같은 detector (같은 카메라 TF, marker 크기, 내부값) 로 dock 의 자리도 잡으므로,
  // 끝에서 detector 가 가르칠 때와 같은 것을 보면 로봇은 가르친 자리에 있다: 그 값들의 오차가 서로 지워진다
  YAML::Node y;
  try {
    y = YAML::LoadFile(file);
  } catch (const std::exception & e) {
    *why = std::string("cannot read ") + file + ": " + e.what();
    return std::nullopt;
  }
  const auto d = y["dock_in_base"];
  if (!d || !d["x"] || !d["y"] || !d["yaw_deg"]) {
    *why = file + " needs dock_in_base: {x, y, yaw_deg} (aruco_teach.py)";
    return std::nullopt;
  }
  if (y["dock"] && y["dock"].as<std::string>() != state_->dock) {
    *why = file + " was taught for dock '" + y["dock"].as<std::string>() + "', not '" + state_->dock + "'";
    return std::nullopt;
  }
  // marker 가 여럿인 dock: 가르칠 때 본 marker 와 지금 설정의 marker 가 같아야 한다 (marker 를 더 붙였으면 다시 가르친다)
  if (const auto configured = cfg_.dock(state_->dock)) {
    std::vector<int> expected = configured->marker_ids, taught;
    if (y["marker_ids"] && y["marker_ids"].IsSequence()) {
      for (const auto & id : y["marker_ids"]) {
        taught.push_back(id.as<int>());
      }
    } else if (expected.size() == 1) {
      taught = expected;                               // 예전 파일 (marker 하나)
    }
    std::sort(expected.begin(), expected.end());
    std::sort(taught.begin(), taught.end());
    if (taught != expected) {
      *why = file + " was taught with other markers than the dock has now";
      return std::nullopt;
    }
  }
  const dock::Pose2 dock_in_base{d["x"].as<double>(), d["y"].as<double>(), d["yaw_deg"].as<double>() * M_PI / 180.0};
  return ad::inverse(dock_in_base);
}

void DetectArucoDock::chooseTarget()
{
  std::string taught;
  if (cfg_.flag("detect", "use_taught", true)) {
    const std::string candidate = DockingConfig::expandHome(
      cfg_.text("detect", "taught_dir", "~/.ros/aruco_taught")) + "/" + state_->dock + ".yaml";
    if (fileExists(candidate)) {
      taught = candidate;
    }
  }
  std::optional<dock::Pose2> target;
  if (!taught.empty()) {
    std::string why;
    target = taughtTarget(taught, &why);
    if (!target) {
      // 예전 형식이거나 다른 dock 의 파일: 도킹을 멈추지 않고 도면 값으로 (다시 가르치라고 알린다)
      RCLCPP_WARN(
        ros_.node->get_logger(), "[aruco] taught target not used (%s): using the geometric target. "
        "Re-teach with aruco_teach.py", why.c_str());
      taught.clear();
    }
  }
  if (target) {
    state_->target = *target;
    state_->target_source = "taught";
  } else {
    if (!cfg_.dock(state_->dock)) {
      RCLCPP_WARN(
        ros_.node->get_logger(), "[aruco] dock '%s' is not in the docking config: using the default gap",
        state_->dock.c_str());
    }
    // 도면 값: 로봇 앞면이 도킹 면에서 gap 만큼 떨어져 설비를 본다 (중심은 gap + robot.front)
    state_->target = {cfg_.dockedDistance(state_->dock), 0.0, M_PI};
    state_->target_source = "geometry";
  }
  RCLCPP_INFO(
    ros_.node->get_logger(), "[aruco] target (%s%s%s): robot center %.1f mm from the docking face "
    "(front edge %.1f mm), %+.1f mm, %.2f deg", state_->target_source.c_str(), taught.empty() ? "" : " ",
    taught.c_str(), state_->target.x * 1000.0, (state_->target.x - cfg_.number("robot", "front", 0.16)) * 1000.0,
    state_->target.y * 1000.0, deg(state_->target.yaw));
}

BT::NodeStatus DetectArucoDock::onStart()
{
  cfg_ = loadDockingConfig(ros_, true);
  ArucoDockStatePtr previous;
  last_dock_.reset();
  if (getInput("state", previous) && previous && previous->filter.initialized()) {
    last_dock_ = previous->filter.mean();             // 다시 시도: 앞선 시도가 잡은 dock 의 자리
  }
  state_ = std::make_shared<ArucoDockState>();
  getInput("target", state_->dock);
  state_->global_frame = cfg_.text("ros", "odom_frame", "odom");
  state_->robot_frame = cfg_.text("ros", "base_frame", "base_footprint");
  bool search = true;
  getInput("search", search);
  search_.clear();
  if (search) {
    for (const double a : cfg_.numbers("detect", "search_angles_deg", {30.0, -30.0, 60.0, -60.0})) {
      search_.push_back(a * M_PI / 180.0);
    }
  }
  settle_time_ = cfg_.number("detect", "settle_time", 0.5);
  look_time_ = cfg_.number("detect", "look_time", 3.0);
  min_detections_ = static_cast<int>(cfg_.number("detect", "min_detections", 8));
  min_found_ = static_cast<int>(cfg_.number("detect", "min_found", 3));
  search_wz_ = cfg_.number("detect", "search_wz", 0.5);
  clearance_ = cfg_.number("detect", "clearance", 0.05);
  timeout_ = cfg_.number("detect", "timeout", 40.0);
  turn_radius_ = cfg_.number("robot", "turn_radius", 0.22);
  if (!cmd_pub_) {
    cmd_pub_ = ros_.node->create_publisher<geometry_msgs::msg::Twist>(
      cfg_.text("ros", "cmd_vel_topic", "cmd_vel_nav"), rclcpp::QoS(10));
  }
  rclcpp::SubscriptionOptions options;
  options.callback_group = ros_.group;
  inbox_.clear();
  seen_.clear();
  other_docks_.clear();
  turn_blocked_.clear();
  received_ = no_tf_ = turns_ = 0;
  next_search_ = 0;
  sub_ = ros_.node->create_subscription<bt_mission_msgs::msg::ArucoDetection>(
    cfg_.text("ros", "detection_topic", "dock/aruco"), rclcpp::QoS(10),
    [this](bt_mission_msgs::msg::ArucoDetection::SharedPtr msg) {inbox_.push_back(*msg);}, options);
  scan_.start(ros_, search_.empty() ? std::string() : cfg_.text("ros", "scan_topic", "scan"));
  const auto robot = ros_.robotPose(state_->global_frame, state_->robot_frame);
  start_yaw_ = robot ? robot->yaw : 0.0;
  start_ = last_cmd_time_ = std::chrono::steady_clock::now();
  enter(Phase::Settle);
  RCLCPP_INFO(
    ros_.node->get_logger(), "[aruco] %s: looking for dock '%s' from here%s", name().c_str(),
    state_->dock.empty() ? "(any)" : state_->dock.c_str(), last_dock_ ? " (retry)" : "");
  return BT::NodeStatus::RUNNING;
}

void DetectArucoDock::enter(Phase phase)
{
  phase_ = phase;
  phase_start_ = std::chrono::steady_clock::now();
  cmd_pub_->publish(toTwist(dock::Cmd{}));
}

void DetectArucoDock::collect(bool keep)
{
  for (const auto & d : inbox_) {
    ++received_;
    if (state_->dock.empty()) {
      state_->dock = d.dock;                          // 처음 보인 dock 으로 정한다
    }
    if (d.dock != state_->dock) {
      if (std::find(other_docks_.begin(), other_docks_.end(), d.dock) == other_docks_.end()) {
        other_docks_.push_back(d.dock);
      }
      continue;
    }
    const auto hs = arucoMeasurements(*ros_.tf, d, state_->global_frame);
    if (hs.empty()) {
      ++no_tf_;
      continue;
    }
    if (keep) {
      seen_.push_back(hs);
    } else {
      seen_.push_back({});                            // 도는 중: 보였다는 것만 (흔들린 영상은 쓰지 않는다)
    }
  }
  inbox_.clear();
}

bool DetectArucoDock::tryFinish()
{
  int used = 0;
  const auto fused = ad::fuseDetections(seen_, kGate, &used);
  if (!fused || used < std::max(min_found_, static_cast<int>(seen_.size()) / 2)) {
    return false;
  }
  chooseTarget();
  state_->filter.reset(fused->pose, fused->cov);
  setOutput("state", state_);
  setOutput("dock_pose", toPoseStamped(fused->pose, state_->global_frame));
  RCLCPP_INFO(
    ros_.node->get_logger(), "[aruco] found '%s' at (%.3f, %.3f), facing %.2f deg, %s (%d of %zu, %d turn(s), %.1f s)",
    state_->dock.c_str(), fused->pose.x, fused->pose.y, deg(fused->pose.yaw), sigmas(fused->cov).c_str(), used,
    seen_.size(), turns_, secondsSince(start_));
  sub_.reset();
  scan_.stop();
  return true;
}

bool DetectArucoDock::turningIsSafe(std::string * why)
{
  if (last_dock_) {
    const auto robot = ros_.robotPose(state_->global_frame, state_->robot_frame);
    if (robot && std::hypot(robot->x - last_dock_->x, robot->y - last_dock_->y) < turn_radius_ + clearance_ + 0.10) {
      *why = "too close to the dock to turn in place";
      return false;
    }
  }
  const auto points = scan_.points(*ros_.tf, state_->robot_frame, why);
  if (!points) {
    return false;
  }
  const int inside = dock::pointsInCircle(*points, turn_radius_ + clearance_);
  if (inside > 3) {
    char text[120];
    std::snprintf(text, sizeof(text), "%d laser points within %.2f m: something is next to the robot", inside,
      turn_radius_ + clearance_);
    *why = text;
    return false;
  }
  return true;
}

bool DetectArucoDock::startNextTurn()
{
  if (next_search_ >= search_.size() || !turn_blocked_.empty()) {
    return false;
  }
  std::string why;
  if (!turningIsSafe(&why)) {
    turn_blocked_ = why;
    RCLCPP_WARN(ros_.node->get_logger(), "[aruco] not turning to search: %s", why.c_str());
    return false;
  }
  turn_to_ = ad::normalizeAngle(start_yaw_ + search_[next_search_++]);
  ++turns_;
  RCLCPP_INFO(
    ros_.node->get_logger(), "[aruco] not seen: turning in place to %+.0f deg from the start heading",
    deg(search_[next_search_ - 1]));
  enter(Phase::Turn);
  return true;
}

BT::NodeStatus DetectArucoDock::fail(const std::string & why)
{
  cmd_pub_->publish(toTwist(dock::Cmd{}));
  std::string hint;
  if (received_ == 0 && sub_ && sub_->get_publisher_count() == 0) {
    hint = "aruco_detector is not running (simple_line.launch.py robot:=mecanum dock:=True starts it, or "
      "ros2 launch bt_mission aruco_detector.launch.py). If it was started, look at its terminal for an error";
  } else if (received_ == 0) {
    hint = "aruco_detector is running but saw no marker: the marker is not in the camera view from here "
      "(is the staging pose in front of the dock and facing it?), or the camera is not publishing";
  } else if (!other_docks_.empty() && seen_.empty()) {
    hint = "saw only other docks:";
    for (const auto & d : other_docks_) {
      hint += " '" + d + "'";
    }
    hint += ". Check the dock name in the task (dock:<name>) and docks in docking.yaml";
  } else if (no_tf_ > 0 && seen_.empty()) {
    hint = std::to_string(no_tf_) + " detections dropped: no TF " + state_->global_frame + " <- " +
      state_->robot_frame + " at the image time";
  } else {
    hint = "only " + std::to_string(seen_.size()) + " usable detections (camera or detector too slow? "
      "check ros2 topic hz /dock/aruco)";
  }
  if (!turn_blocked_.empty()) {
    hint += ". Did not turn to search: " + turn_blocked_;
  }
  sub_.reset();
  scan_.stop();
  RCLCPP_WARN(
    ros_.node->get_logger(), "[aruco] dock '%s' not found (%s) after %.1f s, %d turn(s): %s",
    state_->dock.empty() ? "(any)" : state_->dock.c_str(), why.c_str(), secondsSince(start_), turns_, hint.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DetectArucoDock::onRunning()
{
  ros_.spin();
  if (secondsSince(start_) > timeout_) {
    return fail("timed out");
  }
  if (phase_ == Phase::Settle) {
    inbox_.clear();                                   // 서는 동안의 영상은 버린다
    if (secondsSince(phase_start_) >= settle_time_) {
      seen_.clear();
      enter(Phase::Look);
    }
    return BT::NodeStatus::RUNNING;
  }
  if (phase_ == Phase::Look) {
    collect(true);
    const int n = static_cast<int>(seen_.size());
    if (n >= min_detections_ || (secondsSince(phase_start_) >= look_time_ && n >= min_found_)) {
      if (tryFinish()) {
        return BT::NodeStatus::SUCCESS;
      }
      seen_.clear();                                  // 서로 맞지 않는다: 다른 방향에서 다시
    } else if (secondsSince(phase_start_) < look_time_) {
      return BT::NodeStatus::RUNNING;
    }
    if (received_ == 0 && sub_->get_publisher_count() == 0) {
      return fail("nobody publishes " + std::string(sub_->get_topic_name()));   // 돌아 봐야 소용없다
    }
    if (startNextTurn()) {
      return BT::NodeStatus::RUNNING;
    }
    return fail(search_.empty() ? "not seen from here" : "looked everywhere it could");
  }
  // Phase::Turn: 제자리 회전. marker 가 보이면 그 자리에 선다
  seen_.clear();
  collect(false);
  if (!seen_.empty()) {
    seen_.clear();
    RCLCPP_INFO(ros_.node->get_logger(), "[aruco] marker seen while turning: stopping here to look");
    enter(Phase::Settle);
    return BT::NodeStatus::RUNNING;
  }
  const double dt = secondsSince(last_cmd_time_);
  if (dt < kControlPeriod) {
    return BT::NodeStatus::RUNNING;
  }
  last_cmd_time_ = std::chrono::steady_clock::now();
  std::string why;
  if (!turningIsSafe(&why)) {
    turn_blocked_ = why;
    RCLCPP_WARN(ros_.node->get_logger(), "[aruco] stopped turning: %s", why.c_str());
    enter(Phase::Settle);                             // 여기서 마지막으로 한 번 더 본다
    return BT::NodeStatus::RUNNING;
  }
  const auto robot = ros_.robotPose(state_->global_frame, state_->robot_frame);
  const double error = robot ? ad::normalizeAngle(turn_to_ - robot->yaw) : 0.0;
  if (!robot || std::fabs(error) < 2.0 * M_PI / 180.0 || secondsSince(phase_start_) > 15.0) {
    enter(Phase::Settle);
    return BT::NodeStatus::RUNNING;
  }
  dock::Cmd cmd;
  cmd.wz = std::copysign(std::clamp(1.5 * std::fabs(error), 0.1, search_wz_), error);
  cmd_pub_->publish(toTwist(cmd));
  return BT::NodeStatus::RUNNING;
}

void DetectArucoDock::onHalted()
{
  if (cmd_pub_) {
    cmd_pub_->publish(toTwist(dock::Cmd{}));
  }
  sub_.reset();
  scan_.stop();
}

// ---------------------------------------------------------------------------------------------

ArucoServo::ArucoServo(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), ros_(config)
{
}

BT::NodeStatus ArucoServo::onStart()
{
  if (!getInput("state", state_) || !state_ || !state_->filter.initialized()) {
    RCLCPP_ERROR(ros_.node->get_logger(), "[aruco] %s: no dock estimate (run DetectArucoDock first)", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  const DockingConfig cfg = loadDockingConfig(ros_);
  stage_ = "insert";
  getInput("stage", stage_);
  params_ = cfg.mecanum(stage_);
  const double standoff = cfg.stageNumber(stage_, "standoff", 0.0);
  lost_timeout_ = cfg.stageNumber(stage_, "lost_timeout", 2.0);
  settle_ = cfg.stageNumber(stage_, "settle", 0.3);
  timeout_ = cfg.stageNumber(stage_, "timeout", 30.0);
  drift_ = cfg.stageNumber(stage_, "odom_drift", 0.002);
  slip_ = cfg.stageNumber(stage_, "odom_slip", 0.10);
  yaw_per_m_ = cfg.stageNumber(stage_, "odom_yaw_per_m", 0.05);
  turn_slip_ = cfg.stageNumber(stage_, "odom_turn_slip", 0.05);
  hold_if_closer_ = cfg.stageFlag(stage_, "hold_if_closer", false);
  check_behind_ = cfg.stageFlag(stage_, "check_behind", false);
  fresh_ = cfg.number("safety", "fresh_detection", 0.5);
  min_border_ = cfg.number("safety", "min_border_px", 10.0);
  min_camera_distance_ = cfg.number("safety", "min_camera_distance", 0.10);
  rear_ = cfg.number("robot", "rear", 0.16);
  half_width_ = cfg.number("robot", "half_width", 0.15);
  clearance_ = cfg.number("undock", "clearance", 0.05);
  last_robot_ = ros_.robotPose(state_->global_frame, state_->robot_frame);
  goal_ = {state_->target.x + standoff, state_->target.y, state_->target.yaw};
  if (!cmd_pub_) {
    cmd_pub_ = ros_.node->create_publisher<geometry_msgs::msg::Twist>(
      cfg.text("ros", "cmd_vel_topic", "cmd_vel_nav"), rclcpp::QoS(10));
    estimate_pub_ = ros_.node->create_publisher<geometry_msgs::msg::PoseStamped>("dock/aruco_estimate", 10);
  }
  if (hold_if_closer_ && last_robot_) {
    const ad::DockErrors e = ad::dockErrors(ad::relative(state_->filter.mean(), *last_robot_), goal_);
    if (e.along < -params_.tol_along) {
      // 다시 시도하는 중에 이미 이 단계의 목표보다 설비 가까이 있다: 뒤로 가지 않고 다음 단계(들어가기)로
      RCLCPP_INFO(
        ros_.node->get_logger(), "[aruco] %s: already %.0f mm closer than its goal, not backing up", name().c_str(),
        -e.along * 1000.0);
      return BT::NodeStatus::SUCCESS;
    }
  }
  inbox_.clear();
  rclcpp::SubscriptionOptions options;
  options.callback_group = ros_.group;
  sub_ = ros_.node->create_subscription<bt_mission_msgs::msg::ArucoDetection>(
    cfg.text("ros", "detection_topic", "dock/aruco"), rclcpp::QoS(10),
    [this](bt_mission_msgs::msg::ArucoDetection::SharedPtr msg) {inbox_.push_back(*msg);}, options);
  scan_.start(ros_, check_behind_ ? cfg.text("ros", "scan_topic", "scan") : std::string());
  last_cmd_ = dock::Cmd{};
  accepted_ = rejected_ = held_cycles_ = 0;
  last_border_ = 0.0;
  settled_since_.reset();
  // 이 노드에서 아직 marker 를 보지 못했다: 첫 검출이 오기 전에는 설비 쪽으로 가지 않는다 (seen_marker_ 가 false).
  // 예전에는 last_seen_ 을 지금으로 두어서, 가려진 채 다시 시도할 때마다 처음 0.5 초 동안 앞으로 갔다
  seen_marker_ = false;
  start_ = last_seen_ = last_cmd_time_ = last_predict_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    ros_.node->get_logger(), "[aruco] %s (stage %s): to %.0f mm from the docking face (%s target%s), up to %.2f/%.2f m/s",
    name().c_str(), stage_.c_str(), goal_.x * 1000.0, state_->target_source.c_str(),
    standoff > 0.0 ? " + standoff" : "", params_.max_vx, params_.max_vy);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArucoServo::finish(BT::NodeStatus status, const char * why)
{
  cmd_pub_->publish(toTwist(dock::Cmd{}));
  last_cmd_ = dock::Cmd{};
  sub_.reset();
  scan_.stop();
  const auto robot = ros_.robotPose(state_->global_frame, state_->robot_frame);
  const std::string errors = robot ?
    describe(ad::dockErrors(ad::relative(state_->filter.mean(), *robot), goal_)) : std::string("no robot pose");
  char line[360];
  std::snprintf(
    line, sizeof(line), "[aruco] %s: %s in %.1f s, %s; dock %s (%d detections used, %d rejected, "
    "held %.1f s for the marker)", name().c_str(), why, secondsSince(start_), errors.c_str(),
    sigmas(state_->filter.cov()).c_str(), accepted_, rejected_, held_cycles_ * kControlPeriod);
  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(ros_.node->get_logger(), "%s", line);
  } else {
    RCLCPP_WARN(ros_.node->get_logger(), "%s", line);
  }
  return status;
}

BT::NodeStatus ArucoServo::onRunning()
{
  ros_.spin();
  for (const auto & d : inbox_) {
    if (d.dock != state_->dock) {
      continue;
    }
    if (d.camera_distance > 0.0 && d.camera_distance < min_camera_distance_) {
      inbox_.clear();
      return finish(BT::NodeStatus::FAILURE, "camera too close to the marker (safety.min_camera_distance), stopped");
    }
    const auto hs = arucoMeasurements(*ros_.tf, d, state_->global_frame);
    if (hs.empty()) {
      continue;
    }
    state_->filter.predict(secondsSince(last_predict_), drift_, drift_);
    last_predict_ = std::chrono::steady_clock::now();
    if (const auto robot = ros_.robotPose(state_->global_frame, state_->robot_frame)) {
      if (last_robot_) {
        state_->filter.predictMotion(
          std::hypot(robot->x - last_robot_->x, robot->y - last_robot_->y),
          ad::normalizeAngle(robot->yaw - last_robot_->yaw), slip_, yaw_per_m_, turn_slip_);
      }
      last_robot_ = robot;
    }
    const auto & m = hs[ad::closestHypothesis(state_->filter, hs)];   // 뒤집힌 후보가 있으면 필터에 가까운 쪽
    const auto result = state_->filter.update(m.pose, m.cov, kGate, kReinitAfter);
    if (result == ad::DockFilter::Result::Rejected) {
      ++rejected_;
      continue;
    }
    if (result == ad::DockFilter::Result::Reinitialized) {
      RCLCPP_WARN(ros_.node->get_logger(), "[aruco] %s: dock estimate re-initialized", name().c_str());
    }
    ++accepted_;
    seen_marker_ = true;
    last_seen_ = std::chrono::steady_clock::now();
    last_border_ = d.border_px;
    estimate_pub_->publish(toPoseStamped(state_->filter.mean(), state_->global_frame));
  }
  inbox_.clear();
  if (lost_timeout_ > 0.0 && secondsSince(last_seen_) > lost_timeout_) {
    return finish(BT::NodeStatus::FAILURE, "lost the marker");
  }
  if (secondsSince(start_) > timeout_) {
    return finish(BT::NodeStatus::FAILURE, "timed out");
  }
  const double dt = secondsSince(last_cmd_time_);
  if (dt < kControlPeriod) {
    return BT::NodeStatus::RUNNING;
  }
  last_cmd_time_ = std::chrono::steady_clock::now();
  const auto robot = ros_.robotPose(state_->global_frame, state_->robot_frame);
  if (!robot) {
    cmd_pub_->publish(toTwist(dock::Cmd{}));
    return BT::NodeStatus::RUNNING;
  }
  const dock::Pose2 in_dock = ad::relative(state_->filter.mean(), *robot);
  ad::DockErrors e = ad::dockErrors(in_dock, goal_);
  if (hold_if_closer_) {
    e.along = std::max(e.along, 0.0);              // 설비에서 멀어지는 쪽으로는 가지 않는다
  }
  if (ad::withinTolerance(e, params_)) {
    last_cmd_ = dock::Cmd{};
    cmd_pub_->publish(toTwist(last_cmd_));
    if (!settled_since_) {
      settled_since_ = std::chrono::steady_clock::now();
    }
    return secondsSince(*settled_since_) > settle_ ? finish(BT::NodeStatus::SUCCESS, "reached") :
           BT::NodeStatus::RUNNING;
  }
  settled_since_.reset();
  // 설비 쪽으로 가는 것은 이 노드에서 marker 를 방금 보았고 화면 안쪽에 있을 때만. 아니면 옆과 방향만 고친다.
  // (놓침 판정 lost_timeout 은 시작한 때부터 센다: 처음 검출이 올 때까지의 여유)
  const bool sees = seen_marker_ && secondsSince(last_seen_) <= fresh_ && last_border_ >= min_border_;
  if (!sees && e.along > params_.tol_along) {
    ++held_cycles_;
  }
  dock::Cmd cmd = ad::mecanumStep(e, in_dock.yaw, last_cmd_, params_, std::min(dt, 0.2), sees ? 1.0 : 0.0);
  if (check_behind_ && cmd.vx < -1e-3) {
    // 뒤로 간다: 남은 거리만큼 몸이 쓸고 갈 사각형을 레이저로 본다 (collision_monitor 는 꺼져 있다)
    std::string why;
    const auto points = scan_.points(*ros_.tf, state_->robot_frame, &why);
    if (!points) {
      if (secondsSince(start_) > 2.0) {
        return finish(BT::NodeStatus::FAILURE, "cannot check behind the robot (no laser), stopped");
      }
      cmd = dock::Cmd{};
    } else if (dock::pointsBehind(*points, rear_, half_width_, std::max(0.05, -e.along), clearance_) > 3) {
      return finish(BT::NodeStatus::FAILURE, "something is behind the robot, stopped");
    }
  }
  last_cmd_ = cmd;
  cmd_pub_->publish(toTwist(last_cmd_));
  return BT::NodeStatus::RUNNING;
}

void ArucoServo::onHalted()
{
  if (cmd_pub_) {
    cmd_pub_->publish(toTwist(dock::Cmd{}));
  }
  sub_.reset();
  scan_.stop();
  RCLCPP_WARN(ros_.node->get_logger(), "[aruco] %s: interrupted", name().c_str());
}

// ---------------------------------------------------------------------------------------------

ArucoRefine::ArucoRefine(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), ros_(config)
{
}

void ArucoRefine::enter(Phase phase)
{
  phase_ = phase;
  phase_start_ = std::chrono::steady_clock::now();
  settled_since_.reset();
  inbox_.clear();
  measured_.clear();
  last_cmd_ = dock::Cmd{};
  cmd_pub_->publish(toTwist(last_cmd_));
}

ad::DockErrors ArucoRefine::errorsNow(const dock::Pose2 & robot) const
{
  return ad::dockErrors(ad::relative(state_->filter.mean(), robot), state_->target);
}

BT::NodeStatus ArucoRefine::onStart()
{
  if (!getInput("state", state_) || !state_ || !state_->filter.initialized()) {
    RCLCPP_ERROR(ros_.node->get_logger(), "[aruco] %s: no dock estimate", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  const DockingConfig cfg = loadDockingConfig(ros_);
  std::string stage = "refine";
  getInput("stage", stage);
  params_ = cfg.mecanum(stage);
  settle_time_ = cfg.stageNumber(stage, "settle_time", 0.5);
  measure_time_ = cfg.stageNumber(stage, "measure_time", 1.0);
  max_measure_time_ = cfg.stageNumber(stage, "max_measure_time", 4.0);
  stable_spread_ = cfg.stageNumber(stage, "stable_spread", 0.0015);
  all_markers_ = cfg.stageFlag(stage, "all_markers", true);
  partial_ = 0;
  max_iterations_ = static_cast<int>(cfg.stageNumber(stage, "max_iterations", 3));
  move_timeout_ = cfg.stageNumber(stage, "move_timeout", 8.0);
  accept_along_ = cfg.stageNumber(stage, "accept_along", 0.005);
  accept_lat_ = cfg.stageNumber(stage, "accept_lat", 0.004);
  accept_yaw_ = cfg.stageNumber(stage, "accept_yaw_deg", 1.0) * M_PI / 180.0;
  if (!cmd_pub_) {
    cmd_pub_ = ros_.node->create_publisher<geometry_msgs::msg::Twist>(
      cfg.text("ros", "cmd_vel_topic", "cmd_vel_nav"), rclcpp::QoS(10));
    result_pub_ = ros_.node->create_publisher<geometry_msgs::msg::Vector3Stamped>("dock/final_error", 10);
  }
  rclcpp::SubscriptionOptions options;
  options.callback_group = ros_.group;
  sub_ = ros_.node->create_subscription<bt_mission_msgs::msg::ArucoDetection>(
    cfg.text("ros", "detection_topic", "dock/aruco"), rclcpp::QoS(10),
    [this](bt_mission_msgs::msg::ArucoDetection::SharedPtr msg) {inbox_.push_back(*msg);}, options);
  iteration_ = 0;
  start_ = last_cmd_time_ = std::chrono::steady_clock::now();
  enter(Phase::Settle);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArucoRefine::done(BT::NodeStatus status, const ad::DockErrors & e)
{
  sub_.reset();
  geometry_msgs::msg::Vector3Stamped msg;          // 서서 마지막으로 잰 오차 (기록, 추이 보기)
  msg.header.stamp = ros_.node->now();
  msg.header.frame_id = state_->dock;
  msg.vector.x = e.along;
  msg.vector.y = e.lat;
  msg.vector.z = e.yaw;
  result_pub_->publish(msg);
  return status;
}

BT::NodeStatus ArucoRefine::onRunning()
{
  ros_.spin();
  if (phase_ == Phase::Settle) {
    inbox_.clear();                               // 서는 동안의 영상은 버린다
    if (secondsSince(phase_start_) >= settle_time_) {
      enter(Phase::Measure);
    }
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::Measure) {
    const auto now = std::chrono::steady_clock::now();
    for (const auto & d : inbox_) {
      if (d.dock != state_->dock) {
        continue;
      }
      // marker 가 여럿인 dock: 마지막 판단은 전부 보인 영상으로만 한다. 일부만 보고 푼 자세는 가르칠 때(전부 보고)와
      // 다른 고정 오차를 가진다 (marker 마다 붙인 자리의 오차)
      if (all_markers_ && d.marker_ids.size() < d.markers_total) {
        ++partial_;
        continue;
      }
      const auto hs = arucoMeasurements(*ros_.tf, d, state_->global_frame);
      if (!hs.empty()) {
        measured_.emplace_back(now, hs);
      }
    }
    inbox_.clear();
    const double elapsed = secondsSince(phase_start_);
    if (elapsed < measure_time_) {
      return BT::NodeStatus::RUNNING;
    }
    // 최근 measure_time 동안의 측정만 본다. 그것이 흩어져 있으면 (로봇이 아직 흔들린다) 더 기다린다
    while (!measured_.empty() &&
      std::chrono::duration<double>(now - measured_.front().first).count() > measure_time_)
    {
      measured_.erase(measured_.begin());
    }
    std::vector<ad::Hypotheses> window;
    std::vector<ad::Measurement> first;
    for (const auto & entry : measured_) {
      window.push_back(entry.second);
      first.push_back(entry.second.front());
    }
    const double spread = ad::positionSpread(first);
    if (spread > stable_spread_ && elapsed < max_measure_time_) {
      return BT::NodeStatus::RUNNING;
    }
    if (spread > stable_spread_) {
      RCLCPP_WARN(
        ros_.node->get_logger(), "[aruco] %s: measurements still spread %.1f mm after %.1f s (vibration? "
        "exposure?), using them anyway", name().c_str(), spread * 1000.0, elapsed);
    }
    int used = 0;
    const auto fused = ad::fuseDetections(window, kGate, &used);
    const auto robot = ros_.robotPose(state_->global_frame, state_->robot_frame);
    if (!fused || used < 3 || !robot) {
      sub_.reset();
      RCLCPP_WARN(
        ros_.node->get_logger(), "[aruco] %s: marker not seen while standing (%zu detections%s)", name().c_str(),
        measured_.size(),
        partial_ > 0 ? ", others showed only part of the dock's markers: stages.refine.all_markers" : "");
      return BT::NodeStatus::FAILURE;
    }
    state_->filter.reset(fused->pose, fused->cov);   // 서서 잰 것으로 dock 의 자리를 다시
    const ad::DockErrors e = errorsNow(*robot);
    RCLCPP_INFO(
      ros_.node->get_logger(), "[aruco] %s: measured standing (%d frames in %.1f s, spread %.1f mm, %s): %s",
      name().c_str(), used, elapsed, spread * 1000.0, sigmas(fused->cov).c_str(), describe(e).c_str());
    if (ad::withinTolerance(e, params_)) {
      RCLCPP_INFO(
        ros_.node->get_logger(), "[aruco] %s: done after %d correction(s), %.1f s", name().c_str(), iteration_,
        secondsSince(start_));
      return done(BT::NodeStatus::SUCCESS, e);
    }
    if (iteration_ >= max_iterations_) {
      const bool acceptable = std::fabs(e.along) <= accept_along_ && std::fabs(e.lat) <= accept_lat_ &&
        std::fabs(e.yaw) <= accept_yaw_;
      if (acceptable) {
        RCLCPP_WARN(
          ros_.node->get_logger(), "[aruco] %s: still outside the tolerance after %d corrections, accepted",
          name().c_str(), iteration_);
        return done(BT::NodeStatus::SUCCESS, e);
      }
      RCLCPP_WARN(ros_.node->get_logger(), "[aruco] %s: could not correct it", name().c_str());
      return done(BT::NodeStatus::FAILURE, e);
    }
    ++iteration_;
    enter(Phase::Move);
    return BT::NodeStatus::RUNNING;
  }

  // Phase::Move: 서서 잰 dock 자리는 그대로 두고 odom 으로 고친다. 미끄러져 덜 가면 다음 측정이 잡는다
  inbox_.clear();
  const double dt = secondsSince(last_cmd_time_);
  if (dt < kControlPeriod) {
    return BT::NodeStatus::RUNNING;
  }
  last_cmd_time_ = std::chrono::steady_clock::now();
  const auto robot = ros_.robotPose(state_->global_frame, state_->robot_frame);
  if (!robot) {
    cmd_pub_->publish(toTwist(dock::Cmd{}));
    return BT::NodeStatus::RUNNING;
  }
  const dock::Pose2 in_dock = ad::relative(state_->filter.mean(), *robot);
  const ad::DockErrors e = ad::dockErrors(in_dock, state_->target);
  if (ad::withinTolerance(e, params_) || secondsSince(phase_start_) > move_timeout_) {
    if (!settled_since_) {
      settled_since_ = std::chrono::steady_clock::now();
    }
    last_cmd_ = dock::Cmd{};
    cmd_pub_->publish(toTwist(last_cmd_));
    if (secondsSince(*settled_since_) > 0.2) {
      enter(Phase::Settle);
    }
    return BT::NodeStatus::RUNNING;
  }
  last_cmd_ = ad::mecanumStep(e, in_dock.yaw, last_cmd_, params_, std::min(dt, 0.2));
  cmd_pub_->publish(toTwist(last_cmd_));
  return BT::NodeStatus::RUNNING;
}

void ArucoRefine::onHalted()
{
  if (cmd_pub_) {
    cmd_pub_->publish(toTwist(dock::Cmd{}));
  }
  sub_.reset();
  RCLCPP_WARN(ros_.node->get_logger(), "[aruco] %s: interrupted", name().c_str());
}

void registerArucoDockNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<DetectArucoDock>("DetectArucoDock");
  factory.registerNodeType<ArucoServo>("ArucoServo");
  factory.registerNodeType<ArucoRefine>("ArucoRefine");
}

}  // namespace bt_mission
