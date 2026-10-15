#ifndef SIMPLE_LINE_CONTROLLER__SIMPLE_LINE_CONTROLLER_HPP_
#define SIMPLE_LINE_CONTROLLER__SIMPLE_LINE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"

namespace simple_line_controller
{

// 직선 경로를 따라가는 컨트롤러 (ROS 2 Humble).
//
// 매 주기:
//   1. 받은 경로에서 이미 지나온 부분을 잘라내고,
//      local costmap 범위 안의 경로만 로봇 좌표계로 옮긴다
//      (여러 goal 을 경유하는 꺾은선 경로도 그대로 따라간다)
//   2. goal 의 xy 에 도착했으면       -> 제자리에서 goal yaw 로 회전
//      경로 쪽으로 크게 틀어져 있으면  -> 제자리에서 경로 쪽으로 회전
//      그 외                          -> pure pursuit (곡률 k = 2y / L^2)
//   3. 가속도 제한, goal 근처 감속, 각속도 상한(곡률 유지) 적용
class SimpleLineController : public nav2_core::Controller
{
public:
  SimpleLineController() = default;
  ~SimpleLineController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  bool transformPose(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & in,
    geometry_msgs::msg::PoseStamped & out) const;
  // global 경로를 잘라내고, local costmap 범위 안의 경로를 로봇 좌표계로 반환
  std::vector<Pose2D> localPlan(const Pose2D & robot);
  void publishLocalPlan(
    const std::vector<Pose2D> & local, const builtin_interfaces::msg::Time & stamp);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("SimpleLineController")};

  std::string plan_frame_;
  std::vector<Pose2D> plan_;          // global frame, 지나온 부분은 매 주기 잘라낸다

  // parameters
  double desired_linear_vel_{0.25};
  double max_angular_vel_{1.0};
  double lookahead_dist_{0.6};
  double rotate_to_heading_min_angle_{0.785};
  double rotate_to_heading_angular_vel_{1.0};
  double max_linear_accel_{0.5};
  double max_angular_accel_{3.2};
  double approach_dist_{0.6};
  double min_approach_linear_vel_{0.05};
  double transform_tolerance_{0.1};
  double max_robot_pose_search_dist_{-1.0};   // <= 0 이면 local costmap 반폭

  double control_duration_{0.05};
  double speed_limit_{0.0};           // 0 = 제한 없음
  bool speed_limit_is_percentage_{false};

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  carrot_pub_;
};

}  // namespace simple_line_controller

#endif  // SIMPLE_LINE_CONTROLLER__SIMPLE_LINE_CONTROLLER_HPP_
