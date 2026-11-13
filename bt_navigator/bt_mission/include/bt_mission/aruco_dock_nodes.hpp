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
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

// ArUco marker 하나를 보고 dock 에 붙는 BT 노드들 (task.xml 의 Dock 트리, DOCKING.md).
// 카메라 영상의 marker 는 aruco_detector.py 가 찾아 dock/aruco 로 보낸다: 로봇(base)에서 본 dock 의 바닥 위 자세
// (x, y, yaw) 후보 1 ~ 2 개와 각각의 공분산, 영상을 찍은 시각.
//
//   DetectArucoDock  정지해서 marker 를 여러 번 보고 dock 의 자리(odom)를 잡는다. 설 곳(target)도 정한다:
//                    가르친 자세(taught_file, 없으면 <taught_dir>/<dock>.yaml)가 있으면 그것, 없으면 dock 앞 docked_distance
//   ArucoServo       dock 의 자리를 Kalman 필터로 계속 고치며, dock 좌표계에서 깔때기를 따라 메카넘으로 다가간다
//   ArucoRefine      서서 재고(정지 측정) 고치고를 몇 번 되풀이해 마지막 몇 mm 를 맞춘다
// 물러나기, collision_monitor 끄기, 충전 확인은 dock_nodes.hpp 의 BackOff, MuteCollisionMonitor, CheckCharging.

namespace bt_mission
{

void registerArucoDockNodes(BT::BehaviorTreeFactory & factory);

// 세 노드가 같이 쓰는 것 (blackboard 의 {aruco_dock})
struct ArucoDockState
{
  std::string dock;                     // config/aruco_docks.yaml 의 dock 이름
  std::string global_frame{"odom"};
  std::string robot_frame{"base_footprint"};
  aruco_dock::DockFilter filter;        // dock 좌표계 D 의 자세 (global_frame)
  dock::Pose2 target;                   // 로봇(base)이 설 곳, D 에서
  std::string target_source;            // "geometry" | "taught"
};
using ArucoDockStatePtr = std::shared_ptr<ArucoDockState>;

// detection 하나 -> global frame 의 후보들 (D 의 원점, x 축 방향, 그 공분산). 영상을 찍은 시각의 로봇 자리
// (TF global <- base) 로 옮긴다: 움직이며 찍은 영상도 늦게 도착한 만큼 틀리지 않는다. TF 가 없으면 비어 있다
aruco_dock::Hypotheses arucoMeasurements(
  const tf2_ros::Buffer & tf, const bt_mission_msgs::msg::ArucoDetection & d, const std::string & global);

// 지금 자리에서 marker 를 찾아 dock 의 자리를 잡는다 -> state (filter 를 초기화). 설 곳(target)도 정한다.
// 뒤로 물러나지 않는다 (뒤에 무엇이 있을지 모른다):
//   1. 서서(settle_time) look_time 동안 본다. min_detections 번 보면 끝, look_time 이 지나도 min_found 번 이상이면 끝
//   2. 못 찾으면 search_angles (처음 방향에서 [deg]) 로 차례로 제자리 회전해서 다시 본다 (메카넘은 제자리에서 돈다).
//      돌기 전에, 그리고 도는 동안 레이저로 몸이 쓸고 갈 원(turn_radius + clearance) 안을 본다. 무엇이 있으면 돌지 않는다.
//      앞선 시도에서 dock 의 자리를 알고 dock 이 그 원 가까이 있으면 돌지 않는다 (충전대는 낮아서 레이저에 안 보인다).
//      도는 중에 marker 가 보이면 그 자리에 서서 본다
//   3. 그래도 못 찾으면 FAILURE. 로그에 이유: detector 가 아무것도 안 보냈는지, 다른 dock 만 보였는지, TF 가 없었는지,
//      너무 적게 보였는지, 돌 수 없었는지
class DetectArucoDock : public BT::StatefulActionNode
{
public:
  DetectArucoDock(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target", "", "dock name in aruco_docks.yaml (empty: the first one seen)"),
      BT::InputPort<double>("settle_time", 0.5, "stand still this long before looking [s]"),
      BT::InputPort<double>("look_time", 3.0, "look this long at each heading [s]"),
      BT::InputPort<int>("min_detections", 8, "done as soon as this many are seen"),
      BT::InputPort<int>("min_found", 3, "after look_time, enough if at least this many"),
      BT::InputPort<std::string>("search_angles", "30;-30;60;-60",
        "headings to turn to in place when not seen, from the start heading [deg], ';' separated. empty: no turning"),
      BT::InputPort<double>("search_wz", 0.5, "turning speed [rad/s]"),
      BT::InputPort<double>("turn_radius", 0.22, "robot center to its farthest corner [m]"),
      BT::InputPort<double>("clearance", 0.05, "free margin around the turning circle [m]"),
      BT::InputPort<std::string>("scan_topic", "scan", "laser to check the turning circle. empty: never turn"),
      BT::InputPort<double>("timeout", 40.0, "[s]"),
      BT::InputPort<double>("docked_distance", 0.17, "geometric target: robot center in front of the contact face [m]"),
      BT::InputPort<std::string>("taught_file", "", "yaml from aruco_teach.py (empty: <taught_dir>/<dock>.yaml)"),
      BT::InputPort<std::string>("taught_dir", "~/.ros/aruco_taught", "where aruco_teach.py saves by default"),
      BT::InputPort<bool>("use_taught", true, "false: always the geometric target"),
      BT::InputPort<std::string>("detection_topic", "dock/aruco", ""),
      BT::InputPort<std::string>("global_frame", "odom", ""),
      BT::InputPort<std::string>("robot_frame", "base_footprint", ""),
      BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel_nav", ""),
      BT::BidirectionalPort<ArucoDockStatePtr>("state", "shared with ArucoServo, ArucoRefine (a retry reads the last one)"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("dock_pose", "dock frame origin in the global frame"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase {Settle, Look, Turn};
  void enter(Phase phase);
  void collect(bool keep);              // inbox -> seen_ (keep false: 세기만)
  bool tryFinish();                     // seen_ 으로 dock 의 자리를 잡는다
  bool startNextTurn();                 // 다음 search 방향으로. 돌 수 없거나 남은 방향이 없으면 false
  bool turningIsSafe(std::string * why);
  BT::NodeStatus fail(const std::string & why);
  std::optional<dock::Pose2> taughtTarget(const std::string & file, std::string * why);
  void chooseTarget();                  // dock 을 안 뒤에 설 곳을 정한다

  DockRos ros_;
  rclcpp::Subscription<bt_mission_msgs::msg::ArucoDetection>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  std::chrono::steady_clock::time_point scan_time_;
  std::deque<bt_mission_msgs::msg::ArucoDetection> inbox_;
  std::vector<aruco_dock::Hypotheses> seen_;
  ArucoDockStatePtr state_;
  std::optional<dock::Pose2> last_dock_;   // 앞선 시도가 잡은 dock 의 자리 (있으면)
  Phase phase_{Phase::Settle};
  std::vector<double> search_;             // [rad]
  size_t next_search_{0};
  double start_yaw_{0.0}, turn_to_{0.0};
  double settle_time_{0.5}, look_time_{3.0}, search_wz_{0.5}, turn_radius_{0.22}, clearance_{0.05}, timeout_{40.0};
  int min_detections_{8}, min_found_{3};
  int received_{0}, no_tf_{0}, turns_{0};
  std::vector<std::string> other_docks_;
  std::string turn_blocked_;
  std::chrono::steady_clock::time_point start_, phase_start_, last_cmd_time_;
};

// 공통: 메카넘 파라미터 포트
BT::PortsList mecanumPorts();
aruco_dock::MecanumParams readMecanumParams(const BT::TreeNode & node);

// dock 좌표계에서 target (+ standoff 만큼 방 쪽) 으로 다가간다. 가며 본 marker 로 dock 의 자리를 고친다
class ArucoServo : public BT::StatefulActionNode
{
public:
  ArucoServo(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = mecanumPorts();
    ports.insert(BT::InputPort<ArucoDockStatePtr>("state", ""));
    ports.insert(BT::InputPort<double>("standoff", 0.0, "stop this far before the target, along the dock normal [m]"));
    ports.insert(BT::InputPort<bool>("hold_if_closer", false,
      "true: never move away from the dock. Already closer than the goal (a retry near the dock): succeed at once"));
    ports.insert(BT::InputPort<double>("lost_timeout", 1.0, "fail if no detection is accepted this long, 0: never [s]"));
    ports.insert(BT::InputPort<double>("settle", 0.3, "stay within tolerance this long [s]"));
    ports.insert(BT::InputPort<double>("timeout", 40.0, "[s]"));
    ports.insert(BT::InputPort<double>("odom_drift", 0.002, "random walk of odom [m/sqrt(s)] (and rad/sqrt(s))"));
    ports.insert(BT::InputPort<double>("odom_slip", 0.10, "odom position error per distance moved (mecanum slip)"));
    ports.insert(BT::InputPort<double>("odom_yaw_per_m", 0.05, "odom heading error per metre moved [rad/m]"));
    ports.insert(BT::InputPort<double>("odom_turn_slip", 0.05, "odom heading error per angle turned"));
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
  double slip_{0.10}, yaw_per_m_{0.05}, turn_slip_{0.05};
  bool hold_if_closer_{false};
  std::optional<dock::Pose2> last_robot_;   // 지난 예측 때 로봇의 odom 자세 (움직인 만큼 불확실도를 키운다)
  int accepted_{0}, rejected_{0};
  std::chrono::steady_clock::time_point start_, last_seen_, last_cmd_time_, last_predict_;
  std::optional<std::chrono::steady_clock::time_point> settled_since_;
};

// 서서 재고(measure_time 동안 marker 를 보고 합침), 남은 오차를 odom 으로 고치고, 다시 잰다 (max_iterations 번까지).
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
  std::vector<aruco_dock::Hypotheses> measured_;
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
