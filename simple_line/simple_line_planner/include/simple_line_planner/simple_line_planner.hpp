#ifndef SIMPLE_LINE_PLANNER__SIMPLE_LINE_PLANNER_HPP_
#define SIMPLE_LINE_PLANNER__SIMPLE_LINE_PLANNER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace simple_line_planner
{

// start 에서 goal 까지 직선 경로를 만드는 전역 planner (ROS 2 Humble).
// 장애물은 고려하지 않는다.
//
// - interpolation_resolution 간격으로 점을 찍는다 (간격이 이 값을 넘지 않게 올림)
// - 중간 점의 orientation 은 직선의 진행 방향
// - 마지막 점은 goal 그대로 (goal 의 orientation 유지)
class SimpleLinePlanner : public nav2_core::GlobalPlanner
{
public:
  SimpleLinePlanner() = default;
  ~SimpleLinePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  std::string name_;
  std::string global_frame_;
  double interpolation_resolution_{0.1};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("SimpleLinePlanner")};
};

}  // namespace simple_line_planner

#endif  // SIMPLE_LINE_PLANNER__SIMPLE_LINE_PLANNER_HPP_
