#ifndef SIMPLE_LINE_MISSION__MISSION_NODES_HPP_
#define SIMPLE_LINE_MISSION__MISSION_NODES_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// Nav2 BT 노드들과 같은 PoseStamped 문자열 변환을 쓰도록 포함한다
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace simple_line_mission
{

// mission.xml 에서 쓰는 이 패키지의 BT 노드들을 factory 에 등록한다
void registerNodes(BT::BehaviorTreeFactory & factory);

// ---------------------------------------------------------------------------------------------
// 위치 문자열 처리
// ---------------------------------------------------------------------------------------------

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};   // [rad]
};

// "x,y[,yaw_deg]; x,y[,yaw_deg]; ..." 를 읽는다.
// yaw 를 생략한 점은 진행 방향으로 채운다: 중간 점은 다음 점 쪽, 마지막 점은 이전 점에서 온 방향.
// 형식이 틀리거나, 점이 하나뿐인데 yaw 가 없으면 std::invalid_argument.
std::vector<Pose2D> parsePoses(const std::string & text);

geometry_msgs::msg::PoseStamped toPoseStamped(
  const Pose2D & pose, const std::string & frame, const rclcpp::Time & stamp);

// ---------------------------------------------------------------------------------------------
// BT 노드
//
// SyncActionNode     : tick 한 번에 끝나는 노드. tick() 이 SUCCESS / FAILURE 를 바로 돌려준다.
// StatefulActionNode : 여러 tick 에 걸쳐 실행되는 노드.
//                      onStart() → (RUNNING 이면 다음 tick 마다) onRunning() → SUCCESS / FAILURE.
//                      실행 중에 부모가 멈추라고 하면 onHalted().
// ---------------------------------------------------------------------------------------------

// pose="x,y,yaw_deg" → output (PoseStamped). NavigateToPose 의 goal 로 쓴다.
class MakePose : public BT::SyncActionNode
{
public:
  MakePose(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("pose", "x,y,yaw_deg"),
      BT::InputPort<std::string>("frame", "map", "frame_id of the pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output", "the pose"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

// poses="x,y[,yaw_deg]; ..." → output (std::vector<PoseStamped>). NavigateThroughPoses 의 goals 로 쓴다.
class MakePoses : public BT::SyncActionNode
{
public:
  MakePoses(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("poses", "x,y[,yaw_deg]; x,y[,yaw_deg]; ..."),
      BT::InputPort<std::string>("frame", "map", "frame_id of the poses"),
      // 타입을 적지 않은 포트. Nav2 Humble 1.1.18 이하의 NavigateThroughPoses BT 노드는 goals 포트
      // 타입을 PoseStamped 로 잘못 선언하고 있어서, 여기서 std::vector<PoseStamped> 로 선언하면
      // 트리를 만들 때 타입이 다르다는 오류가 난다 (1.1.19 이후 수정).
      BT::OutputPort("output", "std::vector<geometry_msgs::msg::PoseStamped>"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

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

// message 를 로그로 남기고 SUCCESS.
class Log : public BT::SyncActionNode
{
public:
  Log(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("message")};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace simple_line_mission

#endif  // SIMPLE_LINE_MISSION__MISSION_NODES_HPP_
