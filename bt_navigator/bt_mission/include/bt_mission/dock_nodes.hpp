#ifndef BT_MISSION__DOCK_NODES_HPP_
#define BT_MISSION__DOCK_NODES_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "bt_mission/dock_control.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"

// 도킹에 같이 쓰는 BT 노드들 (task.xml 의 Dock, Undock 트리, DOCKING.md). marker 를 보고 다가가는 노드는
// aruco_dock_nodes.hpp 에 있다.
//
//   BackOff               곧게 뒤로 물러난다 (재시도, undock)
//   MuteCollisionMonitor  자식이 도는 동안만 collision_monitor 의 구역을 끈다 (dock 에 붙을 때)
//   CheckCharging         충전 신호를 기다린다 (토픽을 주지 않으면 건너뛴다)
//
// task_server 는 BT 노드의 콜백을 따로 돌려 주지 않는다. 그래서 노드마다 callback group 과 executor 를
// 두고 tick 할 때 spin_some 한다 (Nav2 의 BT 노드와 같은 방식).

namespace bt_mission
{

void registerDockNodes(BT::BehaviorTreeFactory & factory);

// 노드들이 같이 쓰는 ROS 준비물
struct DockRos
{
  explicit DockRos(const BT::NodeConfiguration & config);
  void spin() {executor.spin_some();}
  std::optional<dock::Pose2> robotPose(const std::string & global, const std::string & robot) const;

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::executors::SingleThreadedExecutor executor;
};

geometry_msgs::msg::PoseStamped toPoseStamped(const dock::Pose2 & pose, const std::string & frame);
dock::Pose2 toPose2(const geometry_msgs::msg::PoseStamped & pose);

// 지금 방향 그대로 distance 만큼 곧게 뒤로 간다 (옆으로 밀리거나 돌면 vy, wz 로 바로잡는다).
// collision_monitor 를 끄고 쓰므로(dock 이 로봇에 붙어 있다) 뒤는 스스로 본다: 남은 거리만큼 몸이 쓸고 갈 사각형
// (robot_rear, robot_half_width, clearance) 안에 레이저 점이 3 개 넘게 있으면 서서 FAILURE
// (fail_when_blocked="false" 이면 거기 선 채로 SUCCESS: 재시도 전에 물러나는 것처럼 못 물러나도 되는 경우).
// scan_topic 을 비우면 보지 않는다. 뒤를 보는 레이저가 없는 로봇이면 뒤로 가는 거리를 짧게 둔다
class BackOff : public BT::StatefulActionNode
{
public:
  BackOff(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", 0.4, "[m]"),
      BT::InputPort<double>("speed", 0.05, "[m/s]"),
      BT::InputPort<double>("timeout", 20.0, "[s]"),
      BT::InputPort<std::string>("scan_topic", "scan", "laser to check behind, empty: no check"),
      BT::InputPort<double>("robot_rear", 0.17, "robot center to its rear edge [m]"),
      BT::InputPort<double>("robot_half_width", 0.16, "[m]"),
      BT::InputPort<double>("clearance", 0.05, "[m]"),
      BT::InputPort<bool>("fail_when_blocked", true, "false: stop where it is and succeed"),
      BT::InputPort<std::string>("global_frame", "odom", ""),
      BT::InputPort<std::string>("robot_frame", "base_footprint", ""),
      BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel_nav", ""),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void publish(const dock::Cmd & cmd);
  BT::NodeStatus stop(BT::NodeStatus status, const char * why);
  int pointsBehind(const sensor_msgs::msg::LaserScan & scan, double distance);

  DockRos ros_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  std::chrono::steady_clock::time_point scan_time_;
  dock::Pose2 goal_;
  dock::ControlParams params_;
  dock::Cmd last_cmd_;
  std::string global_frame_, robot_frame_;
  double timeout_{20.0}, rear_{0.20}, half_width_{0.16}, clearance_{0.05};
  bool fail_when_blocked_{true};
  std::chrono::steady_clock::time_point start_, last_cmd_time_;
};

// 자식이 도는 동안 collision_monitor 의 구역(polygon)을 끈다 (<polygon>.enabled = false).
// 자식이 끝나거나(SUCCESS, FAILURE) 멈춰지면(halt: 작업 취소, 선점) 다시 켠다. 다시 켜지 못하면 FAILURE.
// dock 은 로봇이 붙어야 하는 물체라, 켜 두면 마지막 몇 cm 를 collision_monitor 가 막는다.
// 그동안 로봇은 ArucoServo, ArucoRefine, BackOff 의 낮은 속도로만 움직인다. 끄지 못하면 FAILURE
class MuteCollisionMonitor : public BT::DecoratorNode
{
public:
  MuteCollisionMonitor(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("monitor", "collision_monitor", "collision_monitor node name"),
      BT::InputPort<std::string>("polygon", "Footprint", "polygon to switch off"),
      BT::InputPort<double>("timeout", 2.0, "wait this long for the parameter service [s]"),
    };
  }

  void halt() override;

private:
  BT::NodeStatus tick() override;
  bool setEnabled(bool enabled, std::chrono::milliseconds wait);
  bool restore();                        // 다시 켠다 (응답이 없으면 세 번까지). 못 켜면 false

  enum class State {Idle, Muting, Muted};
  DockRos ros_;
  std::shared_ptr<rclcpp::AsyncParametersClient> client_;
  std::string monitor_, polygon_;
  State state_{State::Idle};
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> pending_;
  std::chrono::steady_clock::time_point start_;
};

// topic(std_msgs/Bool) 에 true 가 오면 SUCCESS (충전 접점이 붙었다). timeout 이 지나면 FAILURE.
// topic 이 비면 확인하지 않고 SUCCESS (시뮬레이션, 충전 신호가 없는 로봇)
class CheckCharging : public BT::StatefulActionNode
{
public:
  CheckCharging(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "", "std_msgs/Bool, true when charging. empty: skip"),
      BT::InputPort<double>("timeout", 5.0, "[s]"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  DockRos ros_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  bool charging_{false};
  double timeout_{5.0};
  std::chrono::steady_clock::time_point start_;
};

}  // namespace bt_mission

#endif  // BT_MISSION__DOCK_NODES_HPP_
