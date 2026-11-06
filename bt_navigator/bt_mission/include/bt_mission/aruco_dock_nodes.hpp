#ifndef BT_MISSION__ARUCO_DOCK_NODES_HPP_
#define BT_MISSION__ARUCO_DOCK_NODES_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "bt_mission/aruco_dock_control.hpp"
#include "bt_mission/dock_nodes.hpp"
#include "bt_mission_msgs/msg/aruco_detection.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

// ArUco board 를 보고 dock 에 붙는 BT 노드들 (task.xml 의 DockAruco 트리, ARUCO_DOCKING.md).
// 카메라 영상의 board 는 aruco_detector.py 가 찾아 dock/aruco 로 보낸다 (자세 + 공분산).
//
//   DetectArucoDock  정지해서 board 를 여러 번 보고 dock 의 자리(odom)를 잡는다. 설 곳(target)도 정한다:
//                    가르친 자세(taught_file, 없으면 <taught_dir>/<board>.yaml)가 있으면 그것, 없으면 dock 앞 docked_distance
//   ArucoServo       dock 의 자리를 Kalman 필터로 계속 고치며, dock 좌표계에서 깔때기를 따라 메카넘으로 다가간다
//   ArucoRefine      서서 재고(정지 측정) 고치고를 몇 번 되풀이해 마지막 몇 mm 를 맞춘다
// 물러나기, collision_monitor 끄기, 충전 확인은 QR 도킹의 BackOff, MuteCollisionMonitor, CheckCharging 을 같이 쓴다.

namespace bt_mission
{

void registerArucoDockNodes(BT::BehaviorTreeFactory & factory);

// 세 노드가 같이 쓰는 것 (blackboard 의 {aruco_dock})
struct ArucoDockState
{
  std::string board;
  std::string global_frame{"odom"};
  std::string robot_frame{"base_footprint"};
  aruco_dock::DockFilter filter;        // dock 좌표계 D 의 자세 (global_frame)
  dock::Pose2 target;                   // 로봇(base)이 설 곳, D 에서
  std::string target_source;            // "geometry" | "taught"
};
using ArucoDockStatePtr = std::shared_ptr<ArucoDockState>;

// detection 하나 -> global frame 의 2D 측정 (D 의 원점, x 축 방향, 그 공분산). TF 가 없거나 판이 서 있지 않으면 없음
std::optional<aruco_dock::Measurement> arucoMeasurement(
  const tf2_ros::Buffer & tf, const bt_mission_msgs::msg::ArucoDetection & d, const std::string & global);

// 정지해서 board 를 min_detections 번 보고 dock 의 자리를 잡는다 -> state (filter 를 초기화)
class DetectArucoDock : public BT::StatefulActionNode
{
public:
  DetectArucoDock(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target", "", "board name (empty: the first one seen)"),
      BT::InputPort<double>("timeout", 5.0, "[s]"),
      BT::InputPort<int>("min_detections", 10, ""),
      BT::InputPort<double>("docked_distance", 0.10, "geometric target: robot center in front of the contact face [m]"),
      BT::InputPort<std::string>("taught_file", "", "yaml from aruco_teach.py (empty: <taught_dir>/<board>.yaml)"),
      BT::InputPort<std::string>("taught_dir", "~/.ros/aruco_taught", "where aruco_teach.py saves by default"),
      BT::InputPort<bool>("use_taught", true, "false: always the geometric target"),
      BT::InputPort<std::string>("detection_topic", "dock/aruco", ""),
      BT::InputPort<std::string>("global_frame", "odom", ""),
      BT::InputPort<std::string>("robot_frame", "base_footprint", ""),
      BT::OutputPort<ArucoDockStatePtr>("state", "shared with ArucoServo, ArucoRefine"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("dock_pose", "dock frame origin in the global frame"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::optional<dock::Pose2> taughtTarget(const std::string & file, std::string * why);
  bool chooseTarget();                  // board 를 안 뒤에 설 곳을 정한다

  DockRos ros_;
  rclcpp::Subscription<bt_mission_msgs::msg::ArucoDetection>::SharedPtr sub_;
  std::deque<bt_mission_msgs::msg::ArucoDetection> inbox_;
  std::vector<aruco_dock::Measurement> seen_;
  ArucoDockStatePtr state_;
  std::chrono::steady_clock::time_point start_;
};

// 공통: 메카넘 파라미터 포트
BT::PortsList mecanumPorts();
aruco_dock::MecanumParams readMecanumParams(const BT::TreeNode & node);

// dock 좌표계에서 target (+ standoff 만큼 방 쪽) 으로 다가간다. 가며 본 board 로 dock 의 자리를 고친다
class ArucoServo : public BT::StatefulActionNode
{
public:
  ArucoServo(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = mecanumPorts();
    ports.insert(BT::InputPort<ArucoDockStatePtr>("state", ""));
    ports.insert(BT::InputPort<double>("standoff", 0.0, "stop this far before the target, along the dock normal [m]"));
    ports.insert(BT::InputPort<double>("lost_timeout", 1.0, "fail if no detection is accepted this long, 0: never [s]"));
    ports.insert(BT::InputPort<double>("settle", 0.3, "stay within tolerance this long [s]"));
    ports.insert(BT::InputPort<double>("timeout", 40.0, "[s]"));
    ports.insert(BT::InputPort<double>("odom_drift", 0.002, "random walk of odom [m/sqrt(s)] (and rad/sqrt(s))"));
    ports.insert(BT::InputPort<std::string>("detection_topic", "dock/aruco", ""));
    ports.insert(BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel_nav", ""));
    return ports;
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus finish(BT::NodeStatus status, const char * why);

  DockRos ros_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimate_pub_;
  rclcpp::Subscription<bt_mission_msgs::msg::ArucoDetection>::SharedPtr sub_;
  std::deque<bt_mission_msgs::msg::ArucoDetection> inbox_;
  ArucoDockStatePtr state_;
  aruco_dock::MecanumParams params_;
  dock::Cmd last_cmd_;
  dock::Pose2 goal_;                    // target + standoff (D)
  double lost_timeout_{1.0}, settle_{0.3}, timeout_{40.0}, drift_{0.002};
  int accepted_{0}, rejected_{0};
  std::chrono::steady_clock::time_point start_, last_seen_, last_cmd_time_, last_predict_;
  std::optional<std::chrono::steady_clock::time_point> settled_since_;
};

// 서서 재고(measure_time 동안 board 를 보고 합침), 남은 오차를 odom 으로 고치고, 다시 잰다 (max_iterations 번까지).
// 움직이는 동안의 영상은 흔들림, 늦음, odom 시각 맞춤 오차가 섞이지만 서서 본 것에는 없다.
// 메카넘은 옆으로 바로 고칠 수 있어서, 차동 구동처럼 물러났다 다시 들어갈 필요가 없다
class ArucoRefine : public BT::StatefulActionNode
{
public:
  ArucoRefine(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = mecanumPorts();
    ports.insert(BT::InputPort<ArucoDockStatePtr>("state", ""));
    ports.insert(BT::InputPort<double>("settle_time", 0.5, "wait after stopping before measuring [s]"));
    ports.insert(BT::InputPort<double>("measure_time", 1.0, "[s]"));
    ports.insert(BT::InputPort<int>("max_iterations", 3, ""));
    ports.insert(BT::InputPort<double>("accept_along", 0.005, "if still outside the tolerance after the last iteration,"
      " succeed when inside these [m]"));
    ports.insert(BT::InputPort<double>("accept_lat", 0.004, "[m]"));
    ports.insert(BT::InputPort<double>("accept_yaw", 1.0, "[deg]"));
    ports.insert(BT::InputPort<double>("move_timeout", 8.0, "[s]"));
    ports.insert(BT::InputPort<std::string>("detection_topic", "dock/aruco", ""));
    ports.insert(BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel_nav", ""));
    return ports;
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase {Settle, Measure, Move};
  void enter(Phase phase);
  aruco_dock::DockErrors errorsNow(const dock::Pose2 & robot) const;

  DockRos ros_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<bt_mission_msgs::msg::ArucoDetection>::SharedPtr sub_;
  std::deque<bt_mission_msgs::msg::ArucoDetection> inbox_;
  std::vector<aruco_dock::Measurement> measured_;
  ArucoDockStatePtr state_;
  aruco_dock::MecanumParams params_;
  dock::Cmd last_cmd_;
  Phase phase_{Phase::Settle};
  int iteration_{0}, max_iterations_{3};
  double settle_time_{0.5}, measure_time_{1.0}, move_timeout_{8.0};
  double accept_along_{0.005}, accept_lat_{0.004}, accept_yaw_{1.0 * M_PI / 180.0};
  std::chrono::steady_clock::time_point start_, phase_start_, last_cmd_time_;
  std::optional<std::chrono::steady_clock::time_point> settled_since_;
};

}  // namespace bt_mission

#endif  // BT_MISSION__ARUCO_DOCK_NODES_HPP_
