#ifndef SIMPLE_LINE_CONTROLLER__SIMPLE_LINE_CONTROLLER_HPP_
#define SIMPLE_LINE_CONTROLLER__SIMPLE_LINE_CONTROLLER_HPP_

#include <memory>
#include <optional>
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
#include "simple_line_controller/motion_profile.hpp"
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
//   3. 직선 속도: 가속 → 순항 → goal 까지 남은 거리로 감속 (S 커브, motion_profile.hpp)
//      경로 위에 장애물(local costmap)이 있으면 그 앞 obstacle_stop_distance 에 서도록 같은 방식으로 감속
//      각속도: 가속도 제한, 제자리 회전은 남은 각도로 감속, 상한에 걸리면 곡률을 유지하며 v 도 줄인다
//   가감속은 바로 전 주기에 낸 명령에서 이어 간다 (odom 의 잡음을 타지 않는다).
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
  // 경로(로봇 좌표계)를 따라 로봇에서 처음으로 막힌 점까지의 거리 [m]. 막힌 점 = 반경
  // obstacle_check_width 안에 local costmap 의 장애물 칸이 있는 점. 없으면 무한대.
  // pose 는 local costmap frame(odom)의 로봇 위치
  double obstacleDistanceOnPath(
    const std::vector<Pose2D> & local, const geometry_msgs::msg::PoseStamped & pose) const;
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
  double rotate_to_heading_tolerance_{0.03};      // 제자리 회전을 끝내는 방향 오차 [rad]
  double rotate_settled_angular_vel_{0.05};       // 회전이 멈췄다고 보는 각속도 [rad/s]
  double max_linear_accel_{0.5};
  double max_linear_decel_{0.5};
  double max_linear_jerk_{0.0};                   // 0 이하면 jerk 제한 없음
  double max_angular_accel_{3.2};
  double min_approach_linear_vel_{0.05};          // goal 바로 앞까지 이 속도로 다가간다
  bool obstacle_check_{true};                     // 경로 위 장애물 앞에서 감속해서 서기
  double obstacle_check_width_{0.3};              // 경로 양옆으로 이만큼 안의 장애물을 본다 [m]
  double obstacle_stop_distance_{0.4};            // 막힌 점 이만큼 앞에 선다 [m]
  double obstacle_resume_delay_{1.0};             // 서서 기다리다가 길이 이만큼 비어 있으면 출발 [s]
  double transform_tolerance_{0.1};
  double max_robot_pose_search_dist_{-1.0};   // <= 0 이면 local costmap 반폭

  double control_duration_{0.05};
  rclcpp::Clock::SharedPtr clock_;
  bool rotating_{false};              // 경로 쪽으로 제자리 회전 중 (방향이 맞을 때까지 계속)
  bool braking_{false};               // goal 앞에서 감속 중
  bool obstacle_braking_{false};      // 경로 위 장애물 앞에서 감속 중 (또는 서서 기다리는 중)
  bool held_{false};                  // 명령을 내는데 로봇이 서 있다 (collision_monitor, 비상정지 등)
  rclcpp::Time held_since_;
  std::optional<rclcpp::Time> clear_since_;   // 경로 위 장애물이 사라진 때 (다시 막히면 비운다)
  // 바로 전 주기에 낸 명령. 가감속은 여기서 이어 간다
  Motion motion_;                     // 직선 속도와 가속도
  double last_w_{0.0};
  rclcpp::Time last_cmd_time_;
  bool has_last_cmd_{false};
  double speed_limit_{0.0};           // 0 = 제한 없음
  bool speed_limit_is_percentage_{false};

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  carrot_pub_;
};

}  // namespace simple_line_controller

#endif  // SIMPLE_LINE_CONTROLLER__SIMPLE_LINE_CONTROLLER_HPP_
