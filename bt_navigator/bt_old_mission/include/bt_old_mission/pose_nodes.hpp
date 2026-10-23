#ifndef BT_OLD_MISSION__POSE_NODES_HPP_
#define BT_OLD_MISSION__POSE_NODES_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
// Nav2 BT 노드들과 같은 PoseStamped 문자열 변환을 쓰도록 포함한다
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "rclcpp/rclcpp.hpp"

// mission.xml 이 좌표를 XML 에 적어 두려고 쓰는 BT 노드 (MakePose, MakePoses).
// 라이브러리 bt_old_mission_nodes 를 플러그인으로 불러오면 등록된다 (BT_REGISTER_NODES).
namespace bt_old_mission
{

// MakePose, MakePoses 를 factory 에 등록한다
void registerPoseNodes(BT::BehaviorTreeFactory & factory);

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

}  // namespace bt_old_mission

#endif  // BT_OLD_MISSION__POSE_NODES_HPP_
