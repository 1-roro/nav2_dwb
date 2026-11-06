#include "bt_mission/aruco_dock_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
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

// 3x3 행렬 (행 우선 9개) 을 R 로 돌린다: R M Rᵀ
std::array<double, 9> rotate(const tf2::Matrix3x3 & r, const std::array<double, 9> & m)
{
  std::array<double, 9> rm{}, out{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        rm[i * 3 + j] += r[i][k] * m[k * 3 + j];
      }
    }
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        out[i * 3 + j] += rm[i * 3 + k] * r[j][k];
      }
    }
  }
  return out;
}

}  // namespace

std::optional<ad::Measurement> arucoMeasurement(
  const tf2_ros::Buffer & tf, const bt_mission_msgs::msg::ArucoDetection & d, const std::string & global)
{
  geometry_msgs::msg::TransformStamped stamped;
  try {
    stamped = tf.lookupTransform(global, d.header.frame_id, d.header.stamp, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException &) {
    return std::nullopt;                          // 영상을 찍은 시각의 로봇 자리
  }
  tf2::Transform global_cam, cam_dock;
  tf2::fromMsg(stamped.transform, global_cam);
  tf2::fromMsg(d.pose.pose, cam_dock);
  const tf2::Transform global_dock = global_cam * cam_dock;
  const tf2::Vector3 x_axis = global_dock.getBasis().getColumn(0);
  if (std::fabs(x_axis.z()) > 0.5) {
    return std::nullopt;                          // 판이 서 있지 않다
  }
  ad::Measurement m;
  m.pose = {global_dock.getOrigin().x(), global_dock.getOrigin().y(), std::atan2(x_axis.y(), x_axis.x())};
  // 공분산: 카메라 frame 의 [위치, 작은 회전] 을 global 로 돌려 (x, y, yaw) 만
  const auto & c = d.pose.covariance;
  std::array<double, 9> pos{}, rot{}, cross{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      pos[i * 3 + j] = c[i * 6 + j];
      rot[i * 3 + j] = c[(i + 3) * 6 + (j + 3)];
      cross[i * 3 + j] = c[i * 6 + (j + 3)];
    }
  }
  const tf2::Matrix3x3 r = global_cam.getBasis();
  const auto p = rotate(r, pos), q = rotate(r, rot), x = rotate(r, cross);
  m.cov = {p[0] + 1e-10, p[1], x[2], p[3], p[4] + 1e-10, x[5], x[2], x[5], q[8] + 1e-10};
  return m;
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
  // aruco_teach.py 가 남긴 것: 제대로 붙은 로봇의 카메라에서 본 D 의 자세. 지금의 카메라 TF(틀려도 된다)로
  // 로봇 자리로 바꾼다. 같은 TF 로 dock 의 자리도 잡으므로, 끝에서 카메라가 같은 것을 보면 로봇은 가르친 자리에 있다
  YAML::Node y;
  try {
    y = YAML::LoadFile(file);
  } catch (const std::exception & e) {
    *why = std::string("cannot read ") + file + ": " + e.what();
    return std::nullopt;
  }
  const auto p = y["position"], q = y["orientation"];
  if (!p || !q || p.size() != 3 || q.size() != 4 || !y["camera_frame"]) {
    *why = file + " needs camera_frame, position [3], orientation [4]";
    return std::nullopt;
  }
  geometry_msgs::msg::TransformStamped base_cam_msg;
  try {
    base_cam_msg = ros_.tf->lookupTransform(
      state_->robot_frame, y["camera_frame"].as<std::string>(), tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    *why = std::string("no transform to the camera: ") + e.what();
    return std::nullopt;
  }
  tf2::Transform base_cam;
  tf2::fromMsg(base_cam_msg.transform, base_cam);
  const tf2::Transform cam_dock(
    tf2::Quaternion(q[0].as<double>(), q[1].as<double>(), q[2].as<double>(), q[3].as<double>()),
    tf2::Vector3(p[0].as<double>(), p[1].as<double>(), p[2].as<double>()));
  const tf2::Transform base_dock = base_cam * cam_dock;
  const tf2::Vector3 x_axis = base_dock.getBasis().getColumn(0);
  const dock::Pose2 dock_in_base{base_dock.getOrigin().x(), base_dock.getOrigin().y(),
    std::atan2(x_axis.y(), x_axis.x())};
  return ad::inverse(dock_in_base);
}

bool DetectArucoDock::chooseTarget()
{
  std::string taught, taught_dir = "~/.ros/aruco_taught";
  bool use_taught = true;
  double docked_distance = 0.10;
  getInput("taught_file", taught);
  getInput("taught_dir", taught_dir);
  getInput("use_taught", use_taught);
  getInput("docked_distance", docked_distance);
  if (taught.empty() && use_taught) {
    const std::string candidate = expandHome(taught_dir) + "/" + state_->board + ".yaml";
    if (fileExists(candidate)) {
      taught = candidate;
    }
  }
  if (use_taught && !taught.empty()) {
    std::string why;
    const auto target = taughtTarget(expandHome(taught), &why);
    if (!target) {
      RCLCPP_ERROR(ros_.node->get_logger(), "[aruco] taught target: %s", why.c_str());
      return false;
    }
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
  return true;
}

BT::NodeStatus DetectArucoDock::onStart()
{
  state_ = std::make_shared<ArucoDockState>();
  std::string topic = "dock/aruco";
  getInput("target", state_->board);
  getInput("global_frame", state_->global_frame);
  getInput("robot_frame", state_->robot_frame);
  getInput("detection_topic", topic);
  inbox_.clear();
  seen_.clear();
  rclcpp::SubscriptionOptions options;
  options.callback_group = ros_.group;
  sub_ = ros_.node->create_subscription<bt_mission_msgs::msg::ArucoDetection>(
    topic, rclcpp::QoS(10),
    [this](bt_mission_msgs::msg::ArucoDetection::SharedPtr msg) {inbox_.push_back(*msg);}, options);
  start_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    ros_.node->get_logger(), "[aruco] looking for board '%s'", state_->board.empty() ? "(any)" : state_->board.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectArucoDock::onRunning()
{
  double timeout = 5.0;
  int min_detections = 10;
  getInput("timeout", timeout);
  getInput("min_detections", min_detections);
  ros_.spin();
  for (const auto & d : inbox_) {
    if (state_->board.empty()) {
      state_->board = d.board;
    }
    if (d.board != state_->board) {
      continue;
    }
    if (const auto m = arucoMeasurement(*ros_.tf, d, state_->global_frame)) {
      seen_.push_back(*m);
    }
  }
  inbox_.clear();
  if (static_cast<int>(seen_.size()) >= min_detections) {
    int used = 0;
    const auto fused = ad::fuse(seen_, kGate, &used);
    if (fused && used >= (min_detections + 1) / 2) {
      sub_.reset();
      if (!chooseTarget()) {
        return BT::NodeStatus::FAILURE;
      }
      state_->filter.reset(fused->pose, fused->cov);
      setOutput("state", state_);
      setOutput("dock_pose", toPoseStamped(fused->pose, state_->global_frame));
      RCLCPP_INFO(
        ros_.node->get_logger(), "[aruco] found '%s' at (%.3f, %.3f), facing %.2f deg, %s (%d of %zu)",
        state_->board.c_str(), fused->pose.x, fused->pose.y, deg(fused->pose.yaw), sigmas(fused->cov).c_str(),
        used, seen_.size());
      sub_.reset();
      return BT::NodeStatus::SUCCESS;
    }
  }
  if (secondsSince(start_) > timeout) {
    RCLCPP_WARN(
      ros_.node->get_logger(), "[aruco] board '%s' not found in %.1f s (%zu detections)",
      state_->board.empty() ? "(any)" : state_->board.c_str(), timeout, seen_.size());
    sub_.reset();
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void DetectArucoDock::onHalted()
{
  sub_.reset();
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
    if (d.board != state_->board) {
      continue;
    }
    const auto m = arucoMeasurement(*ros_.tf, d, state_->global_frame);
    if (!m) {
      continue;
    }
    state_->filter.predict(secondsSince(last_predict_), drift_, drift_);
    last_predict_ = std::chrono::steady_clock::now();
    const auto result = state_->filter.update(m->pose, m->cov, kGate, kReinitAfter);
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
    return finish(BT::NodeStatus::FAILURE, "lost the board");
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
  const ad::DockErrors e = ad::dockErrors(in_dock, goal_);
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
      if (d.board != state_->board) {
        continue;
      }
      if (const auto m = arucoMeasurement(*ros_.tf, d, state_->global_frame)) {
        measured_.push_back(*m);
      }
    }
    inbox_.clear();
    if (secondsSince(phase_start_) < measure_time_) {
      return BT::NodeStatus::RUNNING;
    }
    int used = 0;
    const auto fused = ad::fuse(measured_, kGate, &used);
    const auto robot = ros_.robotPose(state_->global_frame, state_->robot_frame);
    if (!fused || used < 3 || !robot) {
      sub_.reset();
      RCLCPP_WARN(
        ros_.node->get_logger(), "[aruco] %s: board not seen while standing (%zu detections)", name().c_str(),
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
