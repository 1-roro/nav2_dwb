#ifndef STRAIGHT_LINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define STRAIGHT_LINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "straight_line_planner/line_builder.hpp"
#include "tf2_ros/buffer.h"

namespace straight_line_planner
{

// nav2_core::GlobalPlanner 구현 (ROS 2 Humble 인터페이스).
//
// planner_server 가 수명주기(configure -> activate -> deactivate -> cleanup)에 맞춰
// 아래 함수들을 부르고, ComputePathToPose 요청마다 createPlan() 을 부른다.
class StraightLinePlanner : public nav2_core::GlobalPlanner
{
public:
  StraightLinePlanner() = default;
  ~StraightLinePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // Humble 시그니처. (Jazzy 부터는 cancel_checker 인자가 추가된다)
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // P3: 직선이 costmap 의 장애물(LETHAL/INSCRIBED)을 지나면 예외를 던진다.
  void checkCollision(const Pose2D & start, const Pose2D & goal, bool allow_unknown);

  // parent 를 shared_ptr 로 들고 있으면 planner_server 와 순환 참조가 생기므로 weak 로 보관
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("StraightLinePlanner")};
  rclcpp::Clock::SharedPtr clock_;
  int last_step_{-1};
};

}  // namespace straight_line_planner

#endif  // STRAIGHT_LINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
