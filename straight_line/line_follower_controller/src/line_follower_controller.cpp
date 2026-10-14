#include "line_follower_controller/line_follower_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace line_follower_controller
{

namespace
{

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

Pose2D toPose2D(const geometry_msgs::msg::Pose & pose)
{
  return {pose.position.x, pose.position.y, yawOf(pose.orientation)};
}

}  // namespace

void LineFollowerController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("LineFollowerController: unable to lock parent node");
  }
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();

  // 파라미터 선언 ("FollowPath.step" 처럼 플러그인 이름을 앞에 붙인다).
  // 기본값은 Params 구조체의 값을 그대로 쓴다.
  const Params d;
  auto declare = [&](const std::string & key, const rclcpp::ParameterValue & value) {
      nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + "." + key, value);
    };
  declare("step", rclcpp::ParameterValue(d.step));
  declare("desired_linear_vel", rclcpp::ParameterValue(d.desired_linear_vel));
  declare("max_angular_vel", rclcpp::ParameterValue(d.max_angular_vel));
  declare("k_heading", rclcpp::ParameterValue(d.k_heading));
  declare("k_cross_track", rclcpp::ParameterValue(d.k_cross_track));
  declare("lookahead_dist", rclcpp::ParameterValue(d.lookahead_dist));
  declare("rotate_to_heading_min_angle", rclcpp::ParameterValue(d.rotate_to_heading_min_angle));
  declare(
    "rotate_to_heading_angular_vel", rclcpp::ParameterValue(d.rotate_to_heading_angular_vel));
  declare("max_linear_accel", rclcpp::ParameterValue(d.max_linear_accel));
  declare("max_angular_accel", rclcpp::ParameterValue(d.max_angular_accel));
  declare("approach_dist", rclcpp::ParameterValue(d.approach_dist));
  declare("min_approach_linear_vel", rclcpp::ParameterValue(d.min_approach_linear_vel));
  declare("transform_tolerance", rclcpp::ParameterValue(transform_tolerance_));

  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);

  // 제어 주기는 controller_server 의 파라미터에서 가져온다 (RPP 와 같은 방식)
  double controller_frequency = 20.0;
  node->get_parameter("controller_frequency", controller_frequency);
  control_duration_ = 1.0 / controller_frequency;

  // RViz 확인용: 로봇 좌표계로 잘라 낸 경로와 lookahead 점
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("line_follower/local_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "line_follower/carrot", 1);

  RCLCPP_INFO(logger_, "Configured %s (LineFollowerController)", plugin_name_.c_str());
}

void LineFollowerController::cleanup()
{
  local_plan_pub_.reset();
  carrot_pub_.reset();
}

void LineFollowerController::activate()
{
  local_plan_pub_->on_activate();
  carrot_pub_->on_activate();
}

void LineFollowerController::deactivate()
{
  local_plan_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
}

void LineFollowerController::setPlan(const nav_msgs::msg::Path & path)
{
  // 새 경로가 올 때마다 통째로 교체한다. (기본 BT 는 1Hz 로 재계획한다)
  plan_frame_ = path.header.frame_id;
  plan2d_.clear();
  plan2d_.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    plan2d_.push_back(toPose2D(ps.pose));
  }
}

Params LineFollowerController::loadParams(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node) const
{
  // 매 주기 다시 읽는다. `ros2 param set /controller_server FollowPath.step 2` 처럼
  // 실행 중에 단계를 바꿔 볼 수 있게 하기 위해서다.
  Params p;
  auto get = [&](const std::string & key, auto & value) {
      node->get_parameter(plugin_name_ + "." + key, value);
    };
  get("step", p.step);
  get("desired_linear_vel", p.desired_linear_vel);
  get("max_angular_vel", p.max_angular_vel);
  get("k_heading", p.k_heading);
  get("k_cross_track", p.k_cross_track);
  get("lookahead_dist", p.lookahead_dist);
  get("rotate_to_heading_min_angle", p.rotate_to_heading_min_angle);
  get("rotate_to_heading_angular_vel", p.rotate_to_heading_angular_vel);
  get("max_linear_accel", p.max_linear_accel);
  get("max_angular_accel", p.max_angular_accel);
  get("approach_dist", p.approach_dist);
  get("min_approach_linear_vel", p.min_approach_linear_vel);
  return p;
}

geometry_msgs::msg::TwistStamped LineFollowerController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("LineFollowerController: node is gone");
  }

  Params p = loadParams(node);
  if (p.step != last_step_) {
    RCLCPP_INFO(logger_, "[%s] controller step -> %s", plugin_name_.c_str(), stepName(p.step));
    last_step_ = p.step;
    last_state_.clear();
  }

  // speed filter 등이 건 속도 제한 반영
  if (speed_limit_ != nav2_costmap_2d::NO_SPEED_LIMIT) {
    const double limit = speed_limit_is_percentage_ ?
      p.desired_linear_vel * speed_limit_ / 100.0 : speed_limit_;
    p.desired_linear_vel = std::min(p.desired_linear_vel, limit);
  }

  // goal checker 의 xy 허용오차. C3 가 'goal 에 도착했다'고 보는 기준으로 쓴다.
  double goal_dist_tol = 0.25;
  if (goal_checker) {
    geometry_msgs::msg::Pose pose_tol;
    geometry_msgs::msg::Twist vel_tol;
    if (goal_checker->getTolerances(pose_tol, vel_tol)) {
      goal_dist_tol = pose_tol.position.x;
    }
  }

  if (plan2d_.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // 1) 로봇 pose(보통 odom frame)를 경로 frame(보통 map)으로 옮긴다
  geometry_msgs::msg::PoseStamped robot_in_plan;
  if (!transformPose(plan_frame_, pose, robot_in_plan)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }
  const Pose2D robot = toPose2D(robot_in_plan.pose);

  // 2) 지나온 부분 잘라내기 (path pruning)
  const size_t closest = closestIndex(plan2d_, robot);
  plan2d_.erase(plan2d_.begin(), plan2d_.begin() + closest);

  // 3) local costmap 범위 안의 경로만 로봇 좌표계로
  const auto * costmap = costmap_ros_->getCostmap();
  const double max_extent =
    std::max(costmap->getSizeInMetersX(), costmap->getSizeInMetersY()) / 2.0;
  const auto local = localWindow(plan2d_, 0, robot, max_extent);
  const Pose2D goal = toFrame(robot, plan2d_.back());

  // 4) 제어 법칙
  const Command cmd = computeCommand(
    p, local, goal, {velocity.linear.x, velocity.angular.z}, control_duration_, goal_dist_tol);

  publishDebug(local, cmd, pose.header);
  if (last_state_ != cmd.state) {
    RCLCPP_INFO(logger_, "[%s] state -> %s", stepName(p.step), cmd.state);
    last_state_ = cmd.state;
  }
  if (!cmd.ok) {
    // controller_server 는 failure_tolerance(기본 0.3s) 동안 0 속도를 내다가 실패 처리한다
    throw nav2_core::PlannerException(std::string("LineFollowerController: ") + cmd.state);
  }

  geometry_msgs::msg::TwistStamped out;
  out.header = pose.header;
  out.twist.linear.x = cmd.v;
  out.twist.angular.z = cmd.w;
  return out;
}

void LineFollowerController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

bool LineFollowerController::transformPose(
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

void LineFollowerController::publishDebug(
  const std::vector<Pose2D> & local, const Command & cmd,
  const std_msgs::msg::Header & robot_header)
{
  const std::string base_frame = costmap_ros_->getBaseFrameID();

  if (local_plan_pub_->get_subscription_count() > 0) {
    auto msg = std::make_unique<nav_msgs::msg::Path>();
    msg->header.frame_id = base_frame;
    msg->header.stamp = robot_header.stamp;
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

  if (cmd.has_carrot && carrot_pub_->get_subscription_count() > 0) {
    auto msg = std::make_unique<geometry_msgs::msg::PointStamped>();
    msg->header.frame_id = base_frame;
    msg->header.stamp = robot_header.stamp;
    msg->point.x = cmd.carrot.x;
    msg->point.y = cmd.carrot.y;
    carrot_pub_->publish(std::move(msg));
  }
}

}  // namespace line_follower_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(line_follower_controller::LineFollowerController, nav2_core::Controller)
