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

std::string expandHome(const std::string & path)
{
  const char * home = std::getenv("HOME");
  if (!path.empty() && path[0] == '~' && home) {
    return std::string(home) + path.substr(1);
  }
  return path;
}

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

BT::PortsList mecanumPorts()
{
  const ad::MecanumParams d;
  return {
    BT::InputPort<double>("k_along", d.k_along, "[1/s]"),
    BT::InputPort<double>("k_lat", d.k_lat, "[1/s]"),
    BT::InputPort<double>("k_yaw", d.k_yaw, "[1/s]"),
    BT::InputPort<double>("max_vx", d.max_vx, "[m/s]"),
    BT::InputPort<double>("max_vy", d.max_vy, "[m/s]"),
    BT::InputPort<double>("max_wz", d.max_wz, "[rad/s]"),
    BT::InputPort<double>("min_vx", d.min_vx, "deadband [m/s]"),
    BT::InputPort<double>("min_vy", d.min_vy, "deadband [m/s]"),
    BT::InputPort<double>("min_wz", d.min_wz, "deadband [rad/s]"),
    BT::InputPort<double>("accel_x", d.accel_x, "[m/s^2]"),
    BT::InputPort<double>("accel_y", d.accel_y, "[m/s^2]"),
    BT::InputPort<double>("accel_wz", d.accel_wz, "[rad/s^2]"),
    BT::InputPort<double>("scale_vx", d.scale_vx, "commanded / achieved speed, measured on the robot"),
    BT::InputPort<double>("scale_vy", d.scale_vy, ""),
    BT::InputPort<double>("scale_wz", d.scale_wz, ""),
    BT::InputPort<double>("tol_along", d.tol_along, "[m]"),
    BT::InputPort<double>("tol_lat", d.tol_lat, "[m]"),
    BT::InputPort<double>("tol_yaw", deg(d.tol_yaw), "[deg]"),
    BT::InputPort<double>("funnel_lat0", d.funnel_lat0, "[m]"),
    BT::InputPort<double>("funnel_lat_slope", d.funnel_lat_slope, "[m/m]"),
    BT::InputPort<double>("funnel_yaw0", deg(d.funnel_yaw0), "[deg]"),
    BT::InputPort<double>("funnel_yaw_slope", deg(d.funnel_yaw_slope), "[deg/m]"),
  };
}

ad::MecanumParams readMecanumParams(const BT::TreeNode & node)
{
  ad::MecanumParams p;
  node.getInput("k_along", p.k_along);
  node.getInput("k_lat", p.k_lat);
  node.getInput("k_yaw", p.k_yaw);
  node.getInput("max_vx", p.max_vx);
  node.getInput("max_vy", p.max_vy);
  node.getInput("max_wz", p.max_wz);
  node.getInput("min_vx", p.min_vx);
  node.getInput("min_vy", p.min_vy);
  node.getInput("min_wz", p.min_wz);
  node.getInput("accel_x", p.accel_x);
  node.getInput("accel_y", p.accel_y);
  node.getInput("accel_wz", p.accel_wz);
  node.getInput("scale_vx", p.scale_vx);
  node.getInput("scale_vy", p.scale_vy);
  node.getInput("scale_wz", p.scale_wz);
  node.getInput("tol_along", p.tol_along);
  node.getInput("tol_lat", p.tol_lat);
  double v = deg(p.tol_yaw);
  node.getInput("tol_yaw", v);
  p.tol_yaw = v * M_PI / 180.0;
  node.getInput("funnel_lat0", p.funnel_lat0);
  node.getInput("funnel_lat_slope", p.funnel_lat_slope);
  v = deg(p.funnel_yaw0);
  node.getInput("funnel_yaw0", v);
  p.funnel_yaw0 = v * M_PI / 180.0;
  v = deg(p.funnel_yaw_slope);
  node.getInput("funnel_yaw_slope", v);
  p.funnel_yaw_slope = v * M_PI / 180.0;
  return p;
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
  const dock::Pose2 dock_in_base{d["x"].as<double>(), d["y"].as<double>(), d["yaw_deg"].as<double>() * M_PI / 180.0};
  return ad::inverse(dock_in_base);
}

void DetectArucoDock::chooseTarget()
{
  std::string taught, taught_dir = "~/.ros/aruco_taught";
  bool use_taught = true;
  double docked_distance = 0.17;
  getInput("taught_file", taught);
  getInput("taught_dir", taught_dir);
  getInput("use_taught", use_taught);
  getInput("docked_distance", docked_distance);
  if (taught.empty() && use_taught) {
    const std::string candidate = expandHome(taught_dir) + "/" + state_->dock + ".yaml";
    if (fileExists(candidate)) {
      taught = candidate;
    }
  }
  std::optional<dock::Pose2> target;
  if (use_taught && !taught.empty()) {
    std::string why;
    target = taughtTarget(expandHome(taught), &why);
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
    state_->target = {docked_distance, 0.0, M_PI};          // 접점 면 앞 docked_distance 에서 dock 을 본다
    state_->target_source = "geometry";
  }
  RCLCPP_INFO(
    ros_.node->get_logger(), "[aruco] target (%s%s%s): robot %.1f mm in front of the contact face, %+.1f mm, %.2f deg",
    state_->target_source.c_str(), taught.empty() ? "" : " ", taught.c_str(), state_->target.x * 1000.0,
    state_->target.y * 1000.0, deg(state_->target.yaw));
}

namespace
{

std::vector<double> parseAngles(const std::string & text)
{
  std::vector<double> out;
  std::string item;
  for (const char c : text + ";") {
    if (c == ';' || c == ',') {
      if (item.find_first_not_of(" \t") != std::string::npos) {
        out.push_back(std::stod(item) * M_PI / 180.0);
      }
      item.clear();
    } else {
      item += c;
    }
  }
  return out;
}

}  // namespace

BT::NodeStatus DetectArucoDock::onStart()
{
  ArucoDockStatePtr previous;
  last_dock_.reset();
  if (getInput("state", previous) && previous && previous->filter.initialized()) {
    last_dock_ = previous->filter.mean();             // 다시 시도: 앞선 시도가 잡은 dock 의 자리
  }
  state_ = std::make_shared<ArucoDockState>();
  std::string topic = "dock/aruco", cmd_topic = "cmd_vel_nav", scan_topic = "scan", angles = "30;-30;60;-60";
  getInput("target", state_->dock);
  getInput("global_frame", state_->global_frame);
  getInput("robot_frame", state_->robot_frame);
  getInput("detection_topic", topic);
  getInput("cmd_vel_topic", cmd_topic);
  getInput("scan_topic", scan_topic);
  getInput("search_angles", angles);
  getInput("settle_time", settle_time_);
  getInput("look_time", look_time_);
  getInput("min_detections", min_detections_);
  getInput("min_found", min_found_);
  getInput("search_wz", search_wz_);
  getInput("turn_radius", turn_radius_);
  getInput("clearance", clearance_);
  getInput("timeout", timeout_);
  try {
    search_ = parseAngles(angles);
  } catch (const std::exception &) {
    RCLCPP_ERROR(ros_.node->get_logger(), "[aruco] bad search_angles '%s'", angles.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!cmd_pub_) {
    cmd_pub_ = ros_.node->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, rclcpp::QoS(10));
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
    topic, rclcpp::QoS(10),
    [this](bt_mission_msgs::msg::ArucoDetection::SharedPtr msg) {inbox_.push_back(*msg);}, options);
  scan_.reset();
  scan_sub_.reset();
  if (!scan_topic.empty() && !search_.empty()) {
    scan_sub_ = ros_.node->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_ = msg;
        scan_time_ = std::chrono::steady_clock::now();
      }, options);
  }
  const auto robot = ros_.robotPose(state_->global_frame, state_->robot_frame);
  start_yaw_ = robot ? robot->yaw : 0.0;
  start_ = last_cmd_time_ = std::chrono::steady_clock::now();
  enter(Phase::Settle);
  RCLCPP_INFO(
    ros_.node->get_logger(), "[aruco] looking for dock '%s' from here%s", state_->dock.empty() ? "(any)" : state_->dock.c_str(),
    last_dock_ ? " (retry)" : "");
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
  scan_sub_.reset();
  return true;
}

bool DetectArucoDock::turningIsSafe(std::string * why)
{
  if (!scan_sub_) {
    *why = "no laser to check around the robot (scan_topic empty)";
    return false;
  }
  if (last_dock_) {
    const auto robot = ros_.robotPose(state_->global_frame, state_->robot_frame);
    if (robot && std::hypot(robot->x - last_dock_->x, robot->y - last_dock_->y) < turn_radius_ + clearance_ + 0.10) {
      *why = "too close to the dock to turn in place (the charger is too low for the laser)";
      return false;
    }
  }
  if (!scan_ || secondsSince(scan_time_) > 1.0) {
    *why = "no recent laser scan";
    return false;
  }
  const auto points = scanPoints(*ros_.tf, *scan_, state_->robot_frame);
  if (!points) {
    *why = "no transform for the laser";
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
      "ros2 launch bt_mission aruco_detector.launch.py use_sim_time:=True). If it was started, look at its "
      "terminal for an error (e.g. bt_mission_msgs not rebuilt and re-sourced, OpenCV without aruco)";
  } else if (received_ == 0) {
    hint = "aruco_detector is running but saw no marker: the marker is not in the camera view from here "
      "(is the staging pose in front of the dock, 0.5 - 1.5 m, facing it?), or the camera is not publishing "
      "(robot:=mecanum; ros2 topic hz /camera/color/image_raw)";
  } else if (!other_docks_.empty() && seen_.empty()) {
    hint = "saw only other docks:";
    for (const auto & d : other_docks_) {
      hint += " '" + d + "'";
    }
    hint += ". Check the dock name in the task (dock:<name>) and aruco_docks.yaml";
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
  scan_sub_.reset();
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
    return fail(next_search_ >= search_.size() || !turn_blocked_.empty() ? "looked everywhere it could" : "not seen");
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
  scan_sub_.reset();
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
  params_ = readMecanumParams(*this);
  double standoff = 0.0;
  std::string topic = "dock/aruco", cmd_topic = "cmd_vel_nav";
  getInput("standoff", standoff);
  getInput("lost_timeout", lost_timeout_);
  getInput("settle", settle_);
  getInput("timeout", timeout_);
  getInput("odom_drift", drift_);
  getInput("odom_slip", slip_);
  getInput("odom_yaw_per_m", yaw_per_m_);
  getInput("odom_turn_slip", turn_slip_);
  getInput("hold_if_closer", hold_if_closer_);
  last_robot_ = ros_.robotPose(state_->global_frame, state_->robot_frame);
  getInput("detection_topic", topic);
  getInput("cmd_vel_topic", cmd_topic);
  goal_ = {state_->target.x + standoff, state_->target.y, state_->target.yaw};
  if (!cmd_pub_) {
    cmd_pub_ = ros_.node->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, rclcpp::QoS(10));
    estimate_pub_ = ros_.node->create_publisher<geometry_msgs::msg::PoseStamped>("dock/aruco_estimate", 10);
  }
  inbox_.clear();
  rclcpp::SubscriptionOptions options;
  options.callback_group = ros_.group;
  sub_ = ros_.node->create_subscription<bt_mission_msgs::msg::ArucoDetection>(
    topic, rclcpp::QoS(10),
    [this](bt_mission_msgs::msg::ArucoDetection::SharedPtr msg) {inbox_.push_back(*msg);}, options);
  last_cmd_ = dock::Cmd{};
  accepted_ = rejected_ = 0;
  settled_since_.reset();
  if (hold_if_closer_ && last_robot_) {
    const ad::DockErrors e = ad::dockErrors(ad::relative(state_->filter.mean(), *last_robot_), goal_);
    if (e.along < -params_.tol_along) {
      // 다시 시도하는 중에 이미 이 단계의 목표보다 dock 가까이 있다: 뒤로 가지 않고 다음 단계(들어가기)로
      RCLCPP_INFO(
        ros_.node->get_logger(), "[aruco] %s: already %.0f mm closer than its goal, not backing up", name().c_str(),
        -e.along * 1000.0);
      sub_.reset();
      return BT::NodeStatus::SUCCESS;
    }
  }
  start_ = last_seen_ = last_cmd_time_ = last_predict_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    ros_.node->get_logger(), "[aruco] %s: to %.0f mm in front of the contact face (%s target%s), up to %.2f/%.2f m/s",
    name().c_str(), goal_.x * 1000.0, state_->target_source.c_str(),
    standoff > 0.0 ? " + standoff" : "", params_.max_vx, params_.max_vy);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArucoServo::finish(BT::NodeStatus status, const char * why)
{
  cmd_pub_->publish(toTwist(dock::Cmd{}));
  last_cmd_ = dock::Cmd{};
  sub_.reset();
  const auto robot = ros_.robotPose(state_->global_frame, state_->robot_frame);
  const std::string errors = robot ?
    describe(ad::dockErrors(ad::relative(state_->filter.mean(), *robot), goal_)) : std::string("no robot pose");
  char line[300];
  std::snprintf(
    line, sizeof(line), "[aruco] %s: %s in %.1f s, %s; dock %s (%d detections used, %d rejected)", name().c_str(),
    why, secondsSince(start_), errors.c_str(), sigmas(state_->filter.cov()).c_str(), accepted_, rejected_);
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
    last_seen_ = std::chrono::steady_clock::now();
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
    e.along = std::max(e.along, 0.0);              // dock 에서 멀어지는 쪽으로는 가지 않는다
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
  last_cmd_ = ad::mecanumStep(e, in_dock.yaw, last_cmd_, params_, std::min(dt, 0.2));
  cmd_pub_->publish(toTwist(last_cmd_));
  return BT::NodeStatus::RUNNING;
}

void ArucoServo::onHalted()
{
  if (cmd_pub_) {
    cmd_pub_->publish(toTwist(dock::Cmd{}));
  }
  sub_.reset();
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
  params_ = readMecanumParams(*this);
  std::string topic = "dock/aruco", cmd_topic = "cmd_vel_nav";
  getInput("settle_time", settle_time_);
  getInput("measure_time", measure_time_);
  getInput("max_iterations", max_iterations_);
  getInput("move_timeout", move_timeout_);
  getInput("accept_along", accept_along_);
  getInput("accept_lat", accept_lat_);
  double accept_yaw_deg = deg(accept_yaw_);
  getInput("accept_yaw", accept_yaw_deg);
  accept_yaw_ = accept_yaw_deg * M_PI / 180.0;
  getInput("detection_topic", topic);
  getInput("cmd_vel_topic", cmd_topic);
  if (!cmd_pub_) {
    cmd_pub_ = ros_.node->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, rclcpp::QoS(10));
  }
  rclcpp::SubscriptionOptions options;
  options.callback_group = ros_.group;
  sub_ = ros_.node->create_subscription<bt_mission_msgs::msg::ArucoDetection>(
    topic, rclcpp::QoS(10),
    [this](bt_mission_msgs::msg::ArucoDetection::SharedPtr msg) {inbox_.push_back(*msg);}, options);
  iteration_ = 0;
  start_ = last_cmd_time_ = std::chrono::steady_clock::now();
  enter(Phase::Settle);
  return BT::NodeStatus::RUNNING;
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
    for (const auto & d : inbox_) {
      if (d.dock != state_->dock) {
        continue;
      }
      const auto hs = arucoMeasurements(*ros_.tf, d, state_->global_frame);
      if (!hs.empty()) {
        measured_.push_back(hs);
      }
    }
    inbox_.clear();
    if (secondsSince(phase_start_) < measure_time_) {
      return BT::NodeStatus::RUNNING;
    }
    int used = 0;
    const auto fused = ad::fuseDetections(measured_, kGate, &used);
    const auto robot = ros_.robotPose(state_->global_frame, state_->robot_frame);
    if (!fused || used < 3 || !robot) {
      sub_.reset();
      RCLCPP_WARN(
        ros_.node->get_logger(), "[aruco] %s: marker not seen while standing (%zu detections)", name().c_str(),
        measured_.size());
      return BT::NodeStatus::FAILURE;
    }
    state_->filter.reset(fused->pose, fused->cov);   // 서서 잰 것으로 dock 의 자리를 다시
    const ad::DockErrors e = errorsNow(*robot);
    RCLCPP_INFO(
      ros_.node->get_logger(), "[aruco] %s: measured standing (%d frames, %s): %s", name().c_str(), used,
      sigmas(fused->cov).c_str(), describe(e).c_str());
    if (ad::withinTolerance(e, params_)) {
      sub_.reset();
      RCLCPP_INFO(
        ros_.node->get_logger(), "[aruco] %s: done after %d correction(s), %.1f s", name().c_str(), iteration_,
        secondsSince(start_));
      return BT::NodeStatus::SUCCESS;
    }
    if (iteration_ >= max_iterations_) {
      sub_.reset();
      const bool acceptable = std::fabs(e.along) <= accept_along_ && std::fabs(e.lat) <= accept_lat_ &&
        std::fabs(e.yaw) <= accept_yaw_;
      if (acceptable) {
        RCLCPP_WARN(
          ros_.node->get_logger(), "[aruco] %s: still outside the tolerance after %d corrections, accepted",
          name().c_str(), iteration_);
        return BT::NodeStatus::SUCCESS;
      }
      RCLCPP_WARN(ros_.node->get_logger(), "[aruco] %s: could not correct it", name().c_str());
      return BT::NodeStatus::FAILURE;
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
