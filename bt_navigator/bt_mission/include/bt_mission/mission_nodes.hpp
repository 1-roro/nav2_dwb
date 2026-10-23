#ifndef BT_MISSION__MISSION_NODES_HPP_
#define BT_MISSION__MISSION_NODES_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// Nav2 BT 노드들과 같은 PoseStamped 문자열 변환을 쓰도록 포함한다
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "bt_mission_msgs/msg/task.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

namespace bt_mission
{

// task_loop.xml, task.xml 에서 쓰는 이 패키지의 BT 노드들을 factory 에 등록한다
void registerNodes(BT::BehaviorTreeFactory & factory);

// 작업 진행 상황을 알리는 토픽 (std_msgs/String, "<작업 id> <상태>")
constexpr char kStatusTopic[] = "mission/status";

// ---------------------------------------------------------------------------------------------
// BT 노드
//
// SyncActionNode     : tick 한 번에 끝나는 노드. tick() 이 SUCCESS / FAILURE 를 바로 돌려준다.
// StatefulActionNode : 여러 tick 에 걸쳐 실행되는 노드.
//                      onStart() → (RUNNING 이면 다음 tick 마다) onRunning() → SUCCESS / FAILURE.
//                      실행 중에 부모가 멈추라고 하면 onHalted().
// ---------------------------------------------------------------------------------------------

// 정지 상태에서 하는 작업 자리. 지금은 duration 초 동안 RUNNING 을 돌려주고 끝난다.
// 실제로는 이 자리에서 서비스 / 액션을 불러 촬영, 집기, IO 등을 한다.
class DoWork : public BT::StatefulActionNode
{
public:
  DoWork(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("task", "name of the task"),
      BT::InputPort<double>("duration", 2.0, "how long the task takes [s]"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::string task_;
  double duration_{0.0};
  std::chrono::steady_clock::time_point start_;
};

// 주행 중에 하는 작업 자리. 로봇이 interval [m] 움직일 때마다 위치를 기록하고 mission/scan_points 로
// 발행한다. 스스로는 끝나지 않고(항상 RUNNING) 부모(Parallel)가 halt 할 때까지 계속한다.
class ScanWhileMoving : public BT::StatefulActionNode
{
public:
  ScanWhileMoving(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("interval", 0.5, "distance between scans [m]"),
      BT::InputPort<std::string>("global_frame", "map", "frame of the recorded poses"),
      BT::InputPort<std::string>("robot_frame", "base_link", "robot base frame"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;
  geometry_msgs::msg::PoseArray scans_;
  double interval_{0.5};
  std::string global_frame_;
  std::string robot_frame_;
};

// message 를 로그로 남기고 mission/status 로도 발행한 뒤 SUCCESS.
// task_id 를 주면 "<task_id> <message>" 로 보낸다.
class Log : public BT::SyncActionNode
{
public:
  Log(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("message"),
      BT::InputPort<std::string>("task_id", "", "prefix: which task this is about"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

// poses 의 점을 순서대로 하나씩 pose 에 넣고 자식(보통 NavigateToPose)을 실행하는 데코레이터.
//   자식 SUCCESS → 그 점에 도착. 로그와 mission/status 로 "<task_id> arrived <i>/<n>" 을 알리고 다음 점으로
//   자식 FAILURE → 바로 FAILURE (남은 점으로는 가지 않는다)
//   마지막 점에 도착하면 SUCCESS
// reached 에는 지금까지 도착한 점 수를 쓴다 (task_server 가 feedback 으로 보낸다).
// 점마다 멈추므로 NavigateThroughPoses 처럼 근처를 지나쳐 가지 않는다.
class ForEachWaypoint : public BT::DecoratorNode
{
public:
  ForEachWaypoint(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      // 타입을 적지 않는다 (std::vector<geometry_msgs::msg::PoseStamped>). Nav2 Humble 1.1.18 이하의
      // NavigateThroughPoses 는 goals 포트를 PoseStamped 로 잘못 선언해서, 같은 blackboard 항목을
      // NavigateThroughPoses 와 같이 쓰면 타입이 다르다는 오류가 난다 (README 3-3)
      BT::InputPort("poses", "points to visit in order"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "the point the child goes to"),
      BT::OutputPort<int>("reached", "how many points have been reached"),
      BT::InputPort<std::string>("task_id", "", "prefix of the arrival report"),
      BT::InputPort<std::string>("robot_frame", "base_link", "robot frame for the arrival error"),
    };
  }

  void halt() override;

private:
  BT::NodeStatus tick() override;
  void reportArrival();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;  // 있으면 도착할 때 실제 위치와의 오차도 알린다
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  std::size_t index_{0};
};

// 토픽(기본 mission/task)으로 들어온 작업(bt_mission_msgs/Task)을 하나 꺼내
// blackboard 에 쓰고 SUCCESS. 작업이 없으면 올 때까지 RUNNING.
// 실행 중에 들어온 작업은 구독 큐(최대 10개)에 쌓여 있다가 차례로 꺼내진다.
class WaitForTask : public BT::StatefulActionNode
{
public:
  WaitForTask(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "mission/task", "bt_mission_msgs/Task topic"),
      // ForEachWaypoint 의 poses 와 같은 이유로 타입을 적지 않는다
      BT::OutputPort("waypoints", "points to visit, in order"),
      BT::OutputPort<std::string>("task_id"),
      BT::OutputPort<std::string>("action"),
      BT::OutputPort<double>("duration"),
      BT::OutputPort<std::string>("obstacle"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  BT::NodeStatus takeTask();
  void report(const std::string & text);

  rclcpp::Node::SharedPtr node_;
  // 구독 콜백은 이 노드를 tick 할 때만 처리한다 (Nav2 BT 노드들과 같은 방식)
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<bt_mission_msgs::msg::Task>::SharedPtr task_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::deque<bt_mission_msgs::msg::Task> queue_;
  int count_{0};
};

// ---------------------------------------------------------------------------------------------
// 장애물 처리
// ---------------------------------------------------------------------------------------------

// 작업의 obstacle 값(빈 값 / stop / avoid / none)을 아래 두 blackboard 값으로 바꾼다.
//   stop_and_go : StopAndGo 를 켤지. stop, 빈 값이면 true
//   nav_bt      : NavigateToPose 가 bt_navigator 에 보낼 navigation BT 파일.
//                 avoid 면 avoid_bt (장애물을 보는 planner / controller 를 부르는 트리), 아니면 빈 값(기본 트리)
// 모르는 값이면 FAILURE.
class ObstacleMode : public BT::SyncActionNode
{
public:
  ObstacleMode(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("mode", "", "stop | avoid | none, empty: stop"),
      BT::InputPort<std::string>(
        "avoid_bt", "navigate_avoid.xml",
        "navigation BT for avoid. A name without '/' is looked up in bt_mission's behavior_trees/"),
      BT::OutputPort<bool>("stop_and_go"),
      BT::OutputPort<std::string>("nav_bt"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

// stop & go. 자식(NavigateToPose)이 가는 길에 장애물이 있으면 자식을 멈추고(Nav2 goal 취소) 기다리다가,
// 길이 resume_delay 초 동안 비어 있으면 자식을 다시 시작한다(같은 goal 을 다시 보낸다).
//   길    : 로봇에서 goal 까지의 직선 가운데 앞쪽 stop_distance 까지, 폭 2 x half_width 인 띠
//   장애물: 레이저(scan_topic)의 점. scan 이 scan_timeout 초 넘게 오지 않으면 막힌 것으로 본다
// 멈춰 있는 동안 blocked 에 true 를 쓰고, mission/status 로 "<task_id> blocked: ..." / "resumed" 를 알린다.
// max_wait > 0 이면 그보다 오래 막혀 있을 때 FAILURE. enabled 가 false 면 아무것도 보지 않고 자식만 실행한다.
//
// Nav2 의 bt_navigator 에 맡기지 않고 여기서 멈추는 이유: simple_line 의 planner / controller 는 장애물을
// 보지 않는다. 또 goal 을 그대로 둔 채 로봇만 세우면 controller_server 의 progress checker 가 10초 뒤
// 실패를 내고, bt_navigator 가 recovery(제자리 회전, 후진)를 시작한다.
class StopAndGo : public BT::DecoratorNode
{
public:
  StopAndGo(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("enabled", true, "false: just run the child"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "where the child is going"),
      BT::InputPort<double>("stop_distance", 0.6, "stop for an obstacle this close ahead [m]"),
      BT::InputPort<double>("half_width", 0.25, "half width of the path [m] (robot radius + margin)"),
      BT::InputPort<double>("resume_delay", 1.0, "the path must stay clear this long [s]"),
      BT::InputPort<double>("max_wait", 0.0, "fail after being blocked this long [s], 0: wait"),
      BT::InputPort<std::string>("scan_topic", "scan", "sensor_msgs/LaserScan topic"),
      BT::InputPort<double>("scan_timeout", 1.0, "no scan for this long counts as blocked [s]"),
      BT::InputPort<std::string>("robot_frame", "base_link", "robot base frame"),
      BT::InputPort<std::string>("task_id", "", "prefix of the status report"),
      BT::OutputPort<bool>("blocked", "true while stopped for an obstacle"),
    };
  }

  void halt() override;

private:
  // 길 검사 결과. blocked 이면 reason 에 이유 ("obstacle 0.52 m ahead", "no scan on scan")
  struct PathCheck
  {
    bool blocked{false};
    std::string reason;
  };

  BT::NodeStatus tick() override;
  PathCheck checkPath(const geometry_msgs::msg::PoseStamped & goal);
  BT::NodeStatus startChild();
  void report(const std::string & text, bool warn);

  enum class State {kIdle, kGoing, kWaiting};

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  // scan 콜백은 이 노드를 tick 할 때만 처리한다 (WaitForTask 와 같은 방식)
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  std::string scan_topic_;
  std::chrono::system_clock::time_point scan_time_;   // scan 을 보낸 시각
  State state_{State::kIdle};
  std::chrono::steady_clock::time_point blocked_since_;
  std::chrono::steady_clock::time_point clear_since_;
  bool clear_{false};                    // 기다리는 동안 길이 비어 있는지 (clear_since_ 부터)
};

}  // namespace bt_mission

#endif  // BT_MISSION__MISSION_NODES_HPP_
