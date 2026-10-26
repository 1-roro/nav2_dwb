#include "simple_line_planner/simple_line_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

namespace simple_line_planner
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

}  // namespace

void SimpleLinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("SimpleLinePlanner: unable to lock parent node");
  }
  name_ = name;
  costmap_ros_ = costmap_ros;
  global_frame_ = costmap_ros->getGlobalFrameID();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  if (interpolation_resolution_ <= 0.0) {
    throw std::runtime_error("SimpleLinePlanner: interpolation_resolution must be positive");
  }
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".fail_on_obstacle", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".fail_on_obstacle", fail_on_obstacle_);
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);

  RCLCPP_INFO(
    logger_, "Configured %s (SimpleLinePlanner), resolution %.2f m, fail_on_obstacle %s",
    name_.c_str(), interpolation_resolution_, fail_on_obstacle_ ? "true" : "false");
}

void SimpleLinePlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", name_.c_str());
}

void SimpleLinePlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", name_.c_str());
}

void SimpleLinePlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s", name_.c_str());
}

nav_msgs::msg::Path SimpleLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // planner_server 가 start/goal 을 global frame 으로 바꿔서 넘겨준다
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
            "SimpleLinePlanner: start/goal must be in '" + global_frame_ + "'");
  }

  const double sx = start.pose.position.x;
  const double sy = start.pose.position.y;
  const double dx = goal.pose.position.x - sx;
  const double dy = goal.pose.position.y - sy;
  const double dist = std::hypot(dx, dy);

  // 진행 방향. start 와 goal 이 같은 위치면 goal 의 방향을 쓴다
  const double heading = dist > 1e-6 ? std::atan2(dy, dx) : yawOf(goal.pose.orientation);
  // 점 간격이 resolution 을 넘지 않도록 올림
  const int n = std::max(1, static_cast<int>(std::ceil(dist / interpolation_resolution_)));

  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;
  path.poses.reserve(n + 1);

  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / n;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = sx + dx * t;
    pose.pose.position.y = sy + dy * t;
    pose.pose.orientation = quatOf(heading);
    path.poses.push_back(pose);
  }

  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header = path.header;
  path.poses.push_back(goal_pose);

  if (fail_on_obstacle_) {
    checkPath(path);
  }
  return path;
}

void SimpleLinePlanner::checkPath(const nav_msgs::msg::Path & path) const
{
  auto * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
  bool leaving_start = true;    // 출발점 쪽에서 이어지는 막힌 칸은 빼고 본다
  for (const auto & pose : path.poses) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
      if (allow_unknown_) {
        continue;               // 지도 밖 = 모르는 곳
      }
      throw nav2_core::PlannerException("SimpleLinePlanner: the line leaves the map");
    }
    const unsigned char cost = costmap->getCost(mx, my);
    const bool unknown = cost == nav2_costmap_2d::NO_INFORMATION;
    const bool blocked = unknown ? !allow_unknown_ :
      cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    if (!blocked) {
      leaving_start = false;
    } else if (!leaving_start) {
      char where[96];
      std::snprintf(
        where, sizeof(where), " at (%.2f, %.2f)", pose.pose.position.x, pose.pose.position.y);
      throw nav2_core::PlannerException(
              std::string("SimpleLinePlanner: the line crosses an obstacle in the map") + where);
    }
  }
}

}  // namespace simple_line_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(simple_line_planner::SimpleLinePlanner, nav2_core::GlobalPlanner)
