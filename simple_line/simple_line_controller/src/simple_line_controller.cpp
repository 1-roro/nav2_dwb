#include "simple_line_controller/simple_line_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace simple_line_controller
{

namespace
{

double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quatOf(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

double sign(double v)
{
  return v >= 0.0 ? 1.0 : -1.0;
}

}  // namespace

void SimpleLineController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("SimpleLineController: unable to lock parent node");
  }
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();

  auto param = [&](const std::string & key, double & value) {
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + key, rclcpp::ParameterValue(value));
      node->get_parameter(plugin_name_ + "." + key, value);
    };
  param("desired_linear_vel", desired_linear_vel_);
  param("max_angular_vel", max_angular_vel_);
  param("lookahead_dist", lookahead_dist_);
  param("rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
  param("rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);
  param("max_linear_accel", max_linear_accel_);
  param("max_angular_accel", max_angular_accel_);
  param("approach_dist", approach_dist_);
  param("min_approach_linear_vel", min_approach_linear_vel_);
  param("transform_tolerance", transform_tolerance_);

  double controller_frequency = 20.0;
  node->get_parameter("controller_frequency", controller_frequency);
  control_duration_ = 1.0 / controller_frequency;

  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("simple_line/local_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("simple_line/carrot", 1);

  RCLCPP_INFO(logger_, "Configured %s (SimpleLineController)", plugin_name_.c_str());
}

void SimpleLineController::cleanup()
{
  local_plan_pub_.reset();
  carrot_pub_.reset();
}

void SimpleLineController::activate()
{
  local_plan_pub_->on_activate();
  carrot_pub_->on_activate();
}

void SimpleLineController::deactivate()
{
  local_plan_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
}

void SimpleLineController::setPlan(const nav_msgs::msg::Path & path)
{
  plan_frame_ = path.header.frame_id;
  plan_.clear();
  plan_.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    plan_.push_back({ps.pose.position.x, ps.pose.position.y, yawOf(ps.pose.orientation)});
  }
}

std::vector<SimpleLineController::Pose2D> SimpleLineController::localPlan(const Pose2D & robot)
{
  // 1) 로봇에 가장 가까운 점 앞까지 잘라낸다 (path pruning)
  size_t closest = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < plan_.size(); ++i) {
    const double d = std::hypot(plan_[i].x - robot.x, plan_[i].y - robot.y);
    if (d < best) {
      best = d;
      closest = i;
    }
  }
  plan_.erase(plan_.begin(), plan_.begin() + static_cast<long>(closest));

  // 2) local costmap 범위 안의 연속 구간만 로봇 좌표계로 옮긴다
  const auto * costmap = costmap_ros_->getCostmap();
  const double max_dist =
    std::max(costmap->getSizeInMetersX(), costmap->getSizeInMetersY()) / 2.0;
  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);

  std::vector<Pose2D> local;
  for (const auto & p : plan_) {
    const double dx = p.x - robot.x;
    const double dy = p.y - robot.y;
    if (std::hypot(dx, dy) > max_dist) {
      break;
    }
    local.push_back({c * dx + s * dy, -s * dx + c * dy, normalizeAngle(p.yaw - robot.yaw)});
  }
  return local;
}

geometry_msgs::msg::TwistStamped SimpleLineController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (plan_.empty()) {
    throw nav2_core::PlannerException("SimpleLineController: received plan with zero length");
  }

  // 로봇 pose(odom)를 경로 frame(map)으로
  geometry_msgs::msg::PoseStamped robot_ps;
  if (!transformPose(plan_frame_, pose, robot_ps)) {
    throw nav2_core::PlannerException("SimpleLineController: unable to transform robot pose");
  }
  const Pose2D robot{
    robot_ps.pose.position.x, robot_ps.pose.position.y, yawOf(robot_ps.pose.orientation)};

  const auto local = localPlan(robot);
  publishLocalPlan(local, pose.header.stamp);
  if (local.empty()) {
    throw nav2_core::PlannerException("SimpleLineController: no path inside the local costmap");
  }

  // goal (경로의 마지막 점)을 로봇 좌표계로
  const Pose2D & g = plan_.back();
  const double gdx = g.x - robot.x;
  const double gdy = g.y - robot.y;
  const double goal_dist = std::hypot(gdx, gdy);
  const double goal_yaw_err = normalizeAngle(g.yaw - robot.yaw);

  // goal checker 의 xy 허용오차 = '도착했다'고 보는 거리
  double goal_tolerance = 0.25;
  if (goal_checker) {
    geometry_msgs::msg::Pose pose_tol;
    geometry_msgs::msg::Twist vel_tol;
    if (goal_checker->getTolerances(pose_tol, vel_tol)) {
      goal_tolerance = pose_tol.position.x;
    }
  }

  // 속도 제한 (speed filter 등)
  double v_max = desired_linear_vel_;
  if (speed_limit_ != nav2_costmap_2d::NO_SPEED_LIMIT) {
    v_max = std::min(
      v_max, speed_limit_is_percentage_ ? desired_linear_vel_ * speed_limit_ / 100.0 :
      speed_limit_);
  }

  // carrot: lookahead_dist 이상 떨어진 첫 점, 없으면 마지막 점
  auto it = std::find_if(
    local.begin(), local.end(), [&](const Pose2D & p) {
      return std::hypot(p.x, p.y) >= lookahead_dist_;
    });
  const Pose2D carrot = (it != local.end()) ? *it : local.back();
  const double carrot_angle = std::atan2(carrot.y, carrot.x);

  const double dt = control_duration_;
  auto limit_w = [&](double w) {
      return std::clamp(
        w, velocity.angular.z - max_angular_accel_ * dt,
        velocity.angular.z + max_angular_accel_ * dt);
    };

  double v = 0.0;
  double w = 0.0;
  if (goal_dist <= goal_tolerance) {
    // goal 의 xy 에 도착: 제자리에서 goal yaw 로 회전
    w = limit_w(sign(goal_yaw_err) * rotate_to_heading_angular_vel_);
  } else if (std::fabs(carrot_angle) > rotate_to_heading_min_angle_) {
    // 경로 쪽으로 크게 틀어져 있음: 제자리에서 먼저 회전
    w = limit_w(sign(carrot_angle) * rotate_to_heading_angular_vel_);
  } else {
    // pure pursuit
    const double L2 = carrot.x * carrot.x + carrot.y * carrot.y;
    const double curvature = L2 > 1e-6 ? 2.0 * carrot.y / L2 : 0.0;

    v = v_max;
    if (goal_dist < approach_dist_) {
      // goal 근처 감속
      v = std::max(std::min(v, min_approach_linear_vel_), v * goal_dist / approach_dist_);
    }
    // 선가속도 제한
    v = std::clamp(
      v, velocity.linear.x - max_linear_accel_ * dt, velocity.linear.x + max_linear_accel_ * dt);
    v = std::max(v, 0.0);

    w = v * curvature;
    // 각속도 상한에 걸리면 v 도 같이 줄여 곡률을 유지
    if (std::fabs(w) > max_angular_vel_) {
      v *= max_angular_vel_ / std::fabs(w);
      w = sign(w) * max_angular_vel_;
    }
  }

  if (carrot_pub_->get_subscription_count() > 0) {
    auto msg = std::make_unique<geometry_msgs::msg::PointStamped>();
    msg->header.frame_id = costmap_ros_->getBaseFrameID();
    msg->header.stamp = pose.header.stamp;
    msg->point.x = carrot.x;
    msg->point.y = carrot.y;
    carrot_pub_->publish(std::move(msg));
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;
  cmd.twist.linear.x = v;
  cmd.twist.angular.z = w;
  return cmd;
}

void SimpleLineController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

bool SimpleLineController::transformPose(
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in,
  geometry_msgs::msg::PoseStamped & out) const
{
  if (in.header.frame_id == frame) {
    out = in;
    return true;
  }
  try {
    tf_->transform(in, out, frame, tf2::durationFromSec(transform_tolerance_));
    out.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

void SimpleLineController::publishLocalPlan(
  const std::vector<Pose2D> & local, const builtin_interfaces::msg::Time & stamp)
{
  if (local_plan_pub_->get_subscription_count() == 0) {
    return;
  }
  auto msg = std::make_unique<nav_msgs::msg::Path>();
  msg->header.frame_id = costmap_ros_->getBaseFrameID();
  msg->header.stamp = stamp;
  for (const auto & p : local) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg->header;
    ps.pose.position.x = p.x;
    ps.pose.position.y = p.y;
    ps.pose.orientation = quatOf(p.yaw);
    msg->poses.push_back(ps);
  }
  local_plan_pub_->publish(std::move(msg));
}

}  // namespace simple_line_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(simple_line_controller::SimpleLineController, nav2_core::Controller)
