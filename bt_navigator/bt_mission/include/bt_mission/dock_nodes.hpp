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
#include "bt_mission_msgs/msg/qr_code.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"

// QR 을 보고 dock 에 붙는 BT 노드들 (task.xml 의 Dock, Undock 트리, DOCKING.md). 카메라 영상의 QR 은 qr_detector.py 가 찾아 dock/qr 로 보낸다.
//
//   DetectDock            QR 을 여러 번 보고 dock 의 자리(odom)를 잡는다
//   ApproachDock          그 자리를 보며 다가간다. 메카넘이라 vx, vy, wz 를 한꺼번에 낸다 (dock_control.hpp)
//   BackOff               곧게 뒤로 물러난다 (재시도, undock)
//   MuteCollisionMonitor  자식이 도는 동안만 collision_monitor 의 구역을 끈다 (dock 에 붙을 때)
//   CheckCharging         충전 신호를 기다린다 (토픽을 주지 않으면 건너뛴다)
//
// task_server 는 BT 노드의 콜백을 따로 돌려 주지 않는다. 그래서 노드마다 callback group 과 executor 를
// 두고 tick 할 때 spin_some 한다 (Nav2 의 BT 노드와 같은 방식).

namespace bt_mission
{

void registerDockNodes(BT::BehaviorTreeFactory & factory);

// dock 의 자리: position = QR 중심, orientation = QR 앞으로 나오는 법선의 방향 (yaw). frame 은 odom.
// odom 은 짧은 시간에는 틀리지 않으므로, 한 번 잡은 자리를 로봇이 움직이는 동안에도 쓸 수 있다

// 노드들이 같이 쓰는 ROS 준비물
struct DockRos
{
  explicit DockRos(const BT::NodeConfiguration & config);
  void spin() {executor.spin_some();}
  std::optional<dock::Pose2> robotPose(const std::string & global, const std::string & robot) const;
  // QR 하나를 global frame 의 dock 자세로. TF 가 없거나 QR 이 서 있지 않으면(법선이 수평이 아니면) 없음
  std::optional<dock::Pose2> qrPose(const bt_mission_msgs::msg::QrCode & qr, const std::string & global) const;

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::executors::SingleThreadedExecutor executor;
};

geometry_msgs::msg::PoseStamped toPoseStamped(const dock::Pose2 & pose, const std::string & frame);
dock::Pose2 toPose2(const geometry_msgs::msg::PoseStamped & pose);

// QR 을 min_detections 번 보고 dock 의 자리를 dock_pose 로 낸다. 위치는 중앙값에서 max_spread, 방향은 중앙값에서
// max_yaw_spread 안에 드는 것만 평균한다 (dock::robustAverage).
// target 이 비면 처음 보인 QR 을 쓴다. timeout 안에 못 보면 FAILURE
class DetectDock : public BT::StatefulActionNode
{
public:
  DetectDock(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target", "", "QR text to look for (empty: the first one seen)"),
      BT::InputPort<double>("timeout", 5.0, "give up after this long [s]"),
      BT::InputPort<int>("min_detections", 8, "detections to average"),
      BT::InputPort<double>("max_spread", 0.03, "detections farther than this from the median are dropped [m]"),
      BT::InputPort<double>("max_yaw_spread", 5.0, "dropped if turned more than this from the median [deg]"),
      BT::InputPort<std::string>("detection_topic", "dock/qr", "bt_mission_msgs/QrCode"),
      BT::InputPort<std::string>("global_frame", "odom", "frame of dock_pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("dock_pose", "QR center, yaw = its front normal"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  DockRos ros_;
  rclcpp::Subscription<bt_mission_msgs::msg::QrCode>::SharedPtr sub_;
  std::deque<bt_mission_msgs::msg::QrCode> inbox_;
  std::vector<dock::Pose2> seen_;
  int depth_normals_{0};
  std::string target_;
  std::string global_frame_;
  std::chrono::steady_clock::time_point start_;
};

// dock_pose 앞 distance 자리(로봇 중심 ↔ QR 면)로 가서 QR 을 바라보고 선다. 가는 동안 QR 이 보이면
// 최근 window 번의 측정(dock::robustAverage)으로 dock_pose 를 고쳐서 다시 낸다. 카메라가 QR 에 blind_distance 보다 가까우면 보이는 것을 믿지 않고
// (QR 이 화면을 넘친다) 고쳐 둔 자리와 odom 으로 마저 간다.
// lost_timeout 동안 QR 을 못 보면 FAILURE (0 이면 보지 못해도 계속). 오차가 허용오차 안에 settle 초 있으면 SUCCESS
class ApproachDock : public BT::StatefulActionNode
{
public:
  ApproachDock(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<geometry_msgs::msg::PoseStamped>("dock_pose", "dock pose, refined while approaching"),
      BT::InputPort<std::string>("target", "", "QR text (empty: any)"),
      BT::InputPort<double>("distance", 0.5, "goal: robot center to the QR plane [m]"),
      BT::InputPort<double>("max_speed", 0.15, "[m/s]"),
      BT::InputPort<double>("max_angular", 0.5, "[rad/s]"),
      BT::InputPort<double>("accel", 0.3, "[m/s^2]"),
      BT::InputPort<double>("xy_tolerance", 0.01, "[m]"),
      BT::InputPort<double>("yaw_tolerance", 1.0, "[deg]"),
      BT::InputPort<double>("blind_distance", 0.3, "camera to QR distance below which detections are ignored [m]"),
      BT::InputPort<double>("lost_timeout", 2.0, "fail if the QR is not seen this long, 0: never [s]"),
      BT::InputPort<int>("window", 10, "recent detections the dock pose is re-estimated from"),
      BT::InputPort<double>("timeout", 40.0, "[s]"),
      BT::InputPort<std::string>("detection_topic", "dock/qr", ""),
      BT::InputPort<std::string>("global_frame", "odom", ""),
      BT::InputPort<std::string>("robot_frame", "base_footprint", ""),
      BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel_nav", "through collision_monitor and velocity_smoother"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void updateEstimate();
  void publish(const dock::Cmd & cmd);
  BT::NodeStatus finish(BT::NodeStatus status, const char * why);

  DockRos ros_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<bt_mission_msgs::msg::QrCode>::SharedPtr sub_;
  std::deque<bt_mission_msgs::msg::QrCode> inbox_;
  std::deque<dock::Pose2> recent_;
  int window_{10};
  dock::Pose2 estimate_;
  dock::ControlParams params_;
  dock::Cmd last_cmd_;
  std::string target_, global_frame_, robot_frame_;
  double distance_{0.5}, blind_distance_{0.3}, lost_timeout_{2.0}, timeout_{40.0};
  int updates_{0};
  std::chrono::steady_clock::time_point start_, last_seen_, last_cmd_time_;
  std::optional<std::chrono::steady_clock::time_point> settled_since_;
};

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
      BT::InputPort<double>("robot_rear", 0.20, "robot center to its rear edge [m]"),
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
// 그동안 로봇은 ApproachDock, BackOff 의 낮은 속도로만 움직인다. 끄지 못하면 FAILURE
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
