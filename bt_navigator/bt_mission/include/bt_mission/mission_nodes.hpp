#ifndef BT_MISSION__MISSION_NODES_HPP_
#define BT_MISSION__MISSION_NODES_HPP_

#include <chrono>
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
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

namespace bt_mission
{

// task.xml 에서 쓰는 이 패키지의 BT 노드들을 factory 에 등록한다
void registerNodes(BT::BehaviorTreeFactory & factory);

// 작업 진행 상황을 알리는 토픽 (std_msgs/String, "<작업 id> <상태>"). ForEachWaypoint 가 도착을 보낸다
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

}  // namespace bt_mission

#endif  // BT_MISSION__MISSION_NODES_HPP_
