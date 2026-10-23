#ifndef BT_MISSION__BT_COMMON_HPP_
#define BT_MISSION__BT_COMMON_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "bt_mission_msgs/msg/task.hpp"

// mission_bt_runner, task_server, task_server_lifecycle 이 같이 쓰는 BT 준비 코드
namespace bt_mission
{

// Nav2 가 제공하는 BT 노드 플러그인 (NavigateToPose, NavigateThroughPoses, Spin, Wait)
const std::vector<std::string> & nav2BtPlugins();

// 미션 BT 에서 쓰는 BT 노드 종류를 모두 factory 에 등록한다.
//   - Nav2 가 제공하는 노드 (NavigateToPose, NavigateThroughPoses, Spin, Wait): 플러그인 .so 를 이름으로
//     불러온다. bt_navigator 의 plugin_lib_names 파라미터와 같은 방식이다
//   - 이 패키지에서 만든 노드 (mission_nodes.hpp): 코드에서 바로 등록한다
void registerAllNodes(BT::BehaviorTreeFactory & factory);

// 노드들이 같이 쓰는 blackboard. Nav2 BT 노드는 생성될 때 node, server_timeout,
// bt_loop_duration, wait_for_service_timeout 을 꼭 찾는다.
BT::Blackboard::Ptr createBlackboard(
  const rclcpp::Node::SharedPtr & node, const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  double tick_hz);

// 작업 하나를 task.xml 의 blackboard 항목 {waypoints} {task_id} {action} {duration} 에 쓴다.
// frame 이 빈 점은 map 으로, id 가 비면 "task<count>" 로 채운다. 쓴 task_id 를 돌려준다.
std::string writeTask(
  const BT::Blackboard::Ptr & blackboard, bt_mission_msgs::msg::Task task, int count);

// ForEachWaypoint 가 blackboard 의 {waypoints_reached} 에 쓴, 지금까지 도착한 점 수 (없으면 0)
int waypointsReached(const BT::Blackboard::Ptr & blackboard);

// 트리가 FAILURE 로 끝났을 때 result 의 message.
// "<실패한 노드> failed", 점을 다 들르지 못했으면 " at waypoint <i>/<n>" (가던 점) 을 붙인다
std::string failureMessage(const std::string & failed_node, int reached, int total);

// '/' 가 없는 이름은 이 패키지의 share/bt_mission/behavior_trees/ 에서 찾는다
std::string resolveTreePath(const std::string & name);

// lifecycle 노드 name 이 active 가 될 때까지 기다린다. stop 이 true 가 되면 false.
// Nav2 BT 노드는 생성될 때 액션 서버를 찾고, active 전에 보낸 goal 은 거절되기 때문이다.
bool waitForActive(
  const rclcpp::Node::SharedPtr & node, const std::string & name, const std::atomic<bool> & stop);

// 상태가 바뀐 BT 노드를 한 줄씩 출력한다 (print 가 true 일 때).
// 지금 RUNNING 인 액션 노드와, 마지막으로 FAILURE 를 낸 액션 노드도 기억한다.
class TransitionLogger : public BT::StatusChangeLogger
{
public:
  TransitionLogger(BT::TreeNode * root, const rclcpp::Logger & logger, bool print);

  void callback(
    BT::Duration, const BT::TreeNode & node,
    BT::NodeStatus prev, BT::NodeStatus status) override;
  void flush() override {}

  std::string running() const;                       // 이름 순: "go_scan, scanning"
  const std::string & lastFailed() const {return last_failed_;}
  void clear();

private:
  rclcpp::Logger logger_;
  bool print_;
  std::vector<std::string> running_;
  std::string last_failed_;
};

}  // namespace bt_mission

#endif  // BT_MISSION__BT_COMMON_HPP_
