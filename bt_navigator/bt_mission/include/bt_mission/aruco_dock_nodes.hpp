#ifndef BT_MISSION__ARUCO_DOCK_NODES_HPP_
#define BT_MISSION__ARUCO_DOCK_NODES_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "bt_mission/aruco_dock_control.hpp"
#include "bt_mission/dock_nodes.hpp"
#include "bt_mission/docking_config.hpp"
#include "bt_mission_msgs/msg/aruco_detection.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// ArUco marker 하나를 보고 설비(dock)에 붙고 떨어지는 BT 노드들 (task.xml 의 Dock, UndockPrecise 트리, DOCKING.md).
// 카메라 영상의 marker 는 aruco_detector.py 가 찾아 dock/aruco 로 보낸다: 로봇(base)에서 본 dock 의 바닥 위 자세
// (x, y, yaw) 후보 1 ~ 2 개와 각각의 공분산, 영상을 찍은 시각.
//
//   DetectArucoDock  지금 자리에서 marker 를 찾아 dock 의 자리(odom)를 잡는다. 설 곳(target)도 정한다
//   ArucoServo       dock 의 자리를 Kalman 필터로 계속 고치며, dock 좌표계에서 메카넘으로 움직인다.
//                    stage 가 무엇을 하는지 정한다: approach, insert (다가가기), retreat (정밀 undock)
//   ArucoRefine      서서 재고(정지 측정) 고치고를 몇 번 되풀이해 마지막 몇 mm 를 맞춘다
//
// 값(속도, 허용오차, 로봇 치수, 토픽)은 XML 이 아니라 config/docking.yaml 에 있다. XML 에는 순서와 stage 이름만 있다.
// 물러나기(단순 undock), collision_monitor 끄기는 dock_nodes.hpp 의 BackOff, MuteCollisionMonitor.

namespace bt_mission
{

void registerArucoDockNodes(BT::BehaviorTreeFactory & factory);

// 세 노드가 같이 쓰는 것 (blackboard 의 {aruco_dock})
struct ArucoDockState
{
  std::string dock;                     // docking.yaml 의 dock 이름
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

// 레이저를 받아 두고, 로봇 둘레를 묻는다 (DetectArucoDock 의 제자리 회전, ArucoServo 의 물러나기)
class ScanWatch
{
public:
  void start(DockRos & ros, const std::string & topic);
  void stop() {sub_.reset();}
  bool active() const {return static_cast<bool>(sub_);}
  // 최근(1 s 안) 레이저의 점들을 로봇 좌표계로. 없으면 why 에 이유
  std::optional<std::vector<dock::Point2>> points(
    const tf2_ros::Buffer & tf, const std::string & robot_frame, std::string * why) const;

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  std::chrono::steady_clock::time_point time_;
};

// 지금 자리에서 marker 를 찾아 dock 의 자리를 잡는다 -> state (filter 를 초기화). 설 곳(target)도 정한다.
// 뒤로 물러나지 않는다 (뒤에 무엇이 있을지 모른다):
//   1. 서서(settle_time) look_time 동안 본다. min_detections 번 보면 끝, look_time 이 지나도 min_found 번 이상이면 끝
//   2. 못 찾으면 (search="true" 일 때) search_angles_deg 로 차례로 제자리 회전해서 다시 본다.
//      돌기 전에, 그리고 도는 동안 레이저로 몸이 쓸고 갈 원(turn_radius + clearance) 안을 본다. 무엇이 있으면 돌지 않는다.
//      앞선 시도에서 dock 의 자리를 알고 dock 이 그 원 가까이 있으면 돌지 않는다.
//      도는 중에 marker 가 보이면 그 자리에 서서 본다
//   3. 그래도 못 찾으면 FAILURE. 로그에 이유 (DOCKING.md 15장)
// 설 곳: 가르친 자세(<taught_dir>/<dock>.yaml, aruco_teach.py)가 있으면 그것, 없으면 도면 값 (dock 의 gap + 로봇 front).
// 값은 docking.yaml 의 detect, robot, ros, docks
class DetectArucoDock : public BT::StatefulActionNode
{
public:
  DetectArucoDock(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target", "", "dock name in docking.yaml (empty: the first one seen)"),
      BT::InputPort<bool>("search", true, "false: never turn in place to look for the marker"),
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
  DockingConfig cfg_;
  rclcpp::Subscription<bt_mission_msgs::msg::ArucoDetection>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  ScanWatch scan_;
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

// dock 좌표계에서 설 곳 + standoff (설비에서 멀어지는 쪽) 로 간다. 가며 본 marker 로 dock 의 자리를 고친다.
// stage (docking.yaml 의 stages.<stage>) 가 속도, 허용오차, standoff 와 아래 동작을 정한다:
//   hold_if_closer  설비에서 멀어지는 쪽으로 가지 않는다. 이미 목표보다 가까우면 바로 SUCCESS (다시 시도할 때)
//   check_behind    뒤로 갈 때 레이저로 뒤를 본다. 막혀 있으면 서서 FAILURE (retreat)
// 가까이 붙을 때의 안전 (docking.yaml 의 safety):
//   - marker 를 fresh_detection 초 안에 보았고 화면 끝에서 min_border_px 안쪽일 때만 설비 쪽으로 간다.
//     아니면 옆과 방향만 고친다 (marker 를 화면 가운데로 되돌린다). lost_timeout 동안 못 보면 FAILURE
//   - 카메라가 marker 에 min_camera_distance 보다 가까워지면 서서 FAILURE
class ArucoServo : public BT::StatefulActionNode
{
public:
  ArucoServo(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<ArucoDockStatePtr>("state", ""),
      BT::InputPort<std::string>("stage", "insert", "stages.<name> in docking.yaml: approach | insert | retreat"),
    };
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
  ScanWatch scan_;
  std::deque<bt_mission_msgs::msg::ArucoDetection> inbox_;
  ArucoDockStatePtr state_;
  aruco_dock::MecanumParams params_;
  dock::Cmd last_cmd_;
  dock::Pose2 goal_;                    // target + standoff (D)
  std::string stage_;
  double lost_timeout_{2.0}, settle_{0.3}, timeout_{30.0}, drift_{0.002};
  double slip_{0.10}, yaw_per_m_{0.05}, turn_slip_{0.05};
  double fresh_{0.5}, min_border_{10.0}, min_camera_distance_{0.10};
  double rear_{0.16}, half_width_{0.15}, clearance_{0.05};
  bool hold_if_closer_{false}, check_behind_{false};
  double last_border_{0.0};
  bool seen_marker_{false};          // 이 노드가 시작한 뒤 marker 를 받아들인 적이 있다
  std::optional<dock::Pose2> last_robot_;   // 지난 예측 때 로봇의 odom 자세 (움직인 만큼 불확실도를 키운다)
  int accepted_{0}, rejected_{0}, held_cycles_{0};
  std::chrono::steady_clock::time_point start_, last_seen_, last_cmd_time_, last_predict_;
  std::optional<std::chrono::steady_clock::time_point> settled_since_;
};

// 서서 재고(measure_time 동안 marker 를 보고 합침), 남은 오차를 odom 으로 고치고, 다시 잰다 (max_iterations 번까지).
// 움직이는 동안의 영상은 흔들림, 늦음, odom 시각 맞춤 오차가 섞이지만 서서 본 것에는 없다.
// 재는 동안 측정이 stable_spread 보다 흩어져 있으면 (로봇이 아직 흔들린다) max_measure_time 까지 더 기다린다.
// 끝나면 마지막으로 잰 오차를 dock/final_error (Vector3Stamped: x 앞뒤 [m], y 옆 [m], z 방향 [rad], frame_id = dock 이름)
// 로 낸다 (기록, 추이 보기). 값은 docking.yaml 의 stages.<stage> (기본 refine)
class ArucoRefine : public BT::StatefulActionNode
{
public:
  ArucoRefine(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<ArucoDockStatePtr>("state", ""),
      BT::InputPort<std::string>("stage", "refine", "stages.<name> in docking.yaml"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase {Settle, Measure, Move};
  void enter(Phase phase);
  aruco_dock::DockErrors errorsNow(const dock::Pose2 & robot) const;
  BT::NodeStatus done(BT::NodeStatus status, const aruco_dock::DockErrors & e);

  DockRos ros_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr result_pub_;
  rclcpp::Subscription<bt_mission_msgs::msg::ArucoDetection>::SharedPtr sub_;
  std::deque<bt_mission_msgs::msg::ArucoDetection> inbox_;
  std::vector<std::pair<std::chrono::steady_clock::time_point, aruco_dock::Hypotheses>> measured_;
  ArucoDockStatePtr state_;
  aruco_dock::MecanumParams params_;
  dock::Cmd last_cmd_;
  Phase phase_{Phase::Settle};
  int iteration_{0}, max_iterations_{3};
  double settle_time_{0.5}, measure_time_{1.0}, max_measure_time_{4.0}, stable_spread_{0.0015}, move_timeout_{8.0};
  bool all_markers_{true};          // marker 가 여럿인 dock: 전부 보인 영상만 쓴다
  int partial_{0};
  double accept_along_{0.005}, accept_lat_{0.004}, accept_yaw_{1.0 * M_PI / 180.0};
  std::chrono::steady_clock::time_point start_, phase_start_, last_cmd_time_;
  std::optional<std::chrono::steady_clock::time_point> settled_since_;
};

}  // namespace bt_mission

#endif  // BT_MISSION__ARUCO_DOCK_NODES_HPP_
