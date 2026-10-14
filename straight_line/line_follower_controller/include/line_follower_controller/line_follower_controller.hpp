#ifndef LINE_FOLLOWER_CONTROLLER__LINE_FOLLOWER_CONTROLLER_HPP_
#define LINE_FOLLOWER_CONTROLLER__LINE_FOLLOWER_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "line_follower_controller/follower_core.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"

namespace line_follower_controller
{

// nav2_core::Controller 구현 (ROS 2 Humble 인터페이스).
//
// controller_server 가 FollowPath 요청을 받으면 setPlan() 으로 경로를 넘기고,
// controller_frequency(기본 20Hz)마다 computeVelocityCommands() 를 부른다.
// 실제 제어 법칙은 follower_core.hpp 에 있다. 이 클래스는 ROS 와의 연결(TF, 파라미터,
// 시각화, 예외)만 담당한다.
class LineFollowerController : public nav2_core::Controller
{
public:
  LineFollowerController() = default;
  ~LineFollowerController() override = default;

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

  // speed filter(costmap filter) 등이 속도 제한을 걸 때 불린다.
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  Params loadParams(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node) const;
  bool transformPose(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & in,
    geometry_msgs::msg::PoseStamped & out) const;
  void publishDebug(
    const std::vector<Pose2D> & local, const Command & cmd,
    const std_msgs::msg::Header & robot_header);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("LineFollowerController")};

  std::string plan_frame_;              // 받은 경로의 frame (보통 map)
  std::vector<Pose2D> plan2d_;          // 받은 경로 (지나온 부분은 매 주기 잘라낸다)

  double control_duration_{0.05};       // 1 / controller_frequency
  double transform_tolerance_{0.1};
  double speed_limit_{0.0};             // 0 = 제한 없음 (nav2_costmap_2d::NO_SPEED_LIMIT)
  bool speed_limit_is_percentage_{false};
  int last_step_{-1};
  std::string last_state_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  carrot_pub_;
};

}  // namespace line_follower_controller

#endif  // LINE_FOLLOWER_CONTROLLER__LINE_FOLLOWER_CONTROLLER_HPP_
