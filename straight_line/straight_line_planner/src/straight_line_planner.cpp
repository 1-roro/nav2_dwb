#include "straight_line_planner/straight_line_planner.hpp"

#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

namespace straight_line_planner
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

Pose2D toPose2D(const geometry_msgs::msg::PoseStamped & ps)
{
  return {ps.pose.position.x, ps.pose.position.y, yawOf(ps.pose.orientation)};
}

}  // namespace

void StraightLinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("StraightLinePlanner: unable to lock parent node");
  }
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // 파라미터는 "<플러그인 이름>.<파라미터>" 로 선언한다. (예: GridBased.step)
  // createPlan() 에서 매번 다시 읽으므로 실행 중에 `ros2 param set` 으로 단계를 바꿀 수 있다.
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".step", rclcpp::ParameterValue(static_cast<int>(ORIENTED)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));

  RCLCPP_INFO(logger_, "Configured %s (StraightLinePlanner)", name_.c_str());
}

void StraightLinePlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", name_.c_str());
}

void StraightLinePlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", name_.c_str());
}

void StraightLinePlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s", name_.c_str());
}

nav_msgs::msg::Path StraightLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("StraightLinePlanner: node is gone");
  }

  int step = ORIENTED;
  double resolution = 0.1;
  bool allow_unknown = true;
  node->get_parameter(name_ + ".step", step);
  node->get_parameter(name_ + ".interpolation_resolution", resolution);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown);
  if (resolution <= 0.0) {
    throw nav2_core::PlannerException("interpolation_resolution must be positive");
  }
  if (step != last_step_) {
    RCLCPP_INFO(logger_, "[%s] planner step -> %s", name_.c_str(), stepName(step));
    last_step_ = step;
  }

  // planner_server 가 start/goal 을 global frame 으로 변환해서 넘겨 주지만,
  // 공식 튜토리얼처럼 한 번 더 확인한다.
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
            "start/goal must be in the global frame '" + global_frame_ + "' (got '" +
            start.header.frame_id + "', '" + goal.header.frame_id + "')");
  }

  const Pose2D s = toPose2D(start);
  const Pose2D g = toPose2D(goal);

  if (step >= COLLISION) {
    checkCollision(s, g, allow_unknown);
  }

  const auto poses = buildLine(s, g, resolution, step);

  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;
  path.poses.reserve(poses.size());
  for (const auto & p : poses) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = p.x;
    ps.pose.position.y = p.y;
    ps.pose.orientation = quatOf(p.yaw);
    path.poses.push_back(ps);
  }
  // 첫/마지막 pose 는 받은 값을 그대로 쓴다 (P0 의 start 는 로봇의 현재 자세).
  if (step <= ENDPOINTS) {
    path.poses.front().pose = start.pose;
  }
  path.poses.back().pose = goal.pose;

  RCLCPP_DEBUG(
    logger_, "[%s] %zu poses, %.2f m", stepName(step), path.poses.size(),
    std::hypot(g.x - s.x, g.y - s.y));
  return path;
}

void StraightLinePlanner::checkCollision(
  const Pose2D & start, const Pose2D & goal, bool allow_unknown)
{
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  bool out_of_bounds = false;
  auto is_blocked = [&](double wx, double wy) {
      unsigned int mx, my;
      if (!costmap_->worldToMap(wx, wy, mx, my)) {
        out_of_bounds = true;
        return true;
      }
      const unsigned char cost = costmap_->getCost(mx, my);
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        return !allow_unknown;
      }
      // INSCRIBED(253) 이상 = 로봇 중심이 그 셀에 오면 몸체가 장애물에 닿는다
      return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    };

  // costmap 해상도의 절반 간격으로 훑어서 셀을 건너뛰지 않게 한다.
  const double sample_step = costmap_->getResolution() * 0.5;
  const double blocked_at = firstBlockedDistance(start, goal, sample_step, is_blocked);
  if (blocked_at < 0.0) {
    return;
  }

  const double total = std::hypot(goal.x - start.x, goal.y - start.y);
  std::string reason;
  if (out_of_bounds) {
    reason = "line leaves the costmap";
  } else if (blocked_at < 1e-6) {
    reason = "start is in collision";
  } else if (total - blocked_at < sample_step) {
    reason = "goal is in collision";
  } else {
    reason = "line is blocked " + std::to_string(blocked_at) + " m from the start";
  }
  RCLCPP_WARN(logger_, "[%s] rejecting straight line: %s", name_.c_str(), reason.c_str());
  throw nav2_core::PlannerException("StraightLinePlanner: " + reason);
}

}  // namespace straight_line_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(straight_line_planner::StraightLinePlanner, nav2_core::GlobalPlanner)
