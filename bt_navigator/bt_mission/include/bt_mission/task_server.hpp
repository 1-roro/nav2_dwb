#ifndef BT_MISSION__TASK_SERVER_HPP_
#define BT_MISSION__TASK_SERVER_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bt_mission/bt_common.hpp"
#include "bt_mission_msgs/action/execute_task.hpp"

namespace bt_mission
{

// ExecuteTask 액션 서버. goal 하나 = 작업 하나를 BT(task.xml)로 실행한다.
//
//   goal 수락   blackboard 에 waypoints / task_id / action / duration 을 쓰고 트리를 처음부터 tick
//   실행 중     feedback: 지금 RUNNING 인 액션 노드, 도착한 점 수, 지난 시간
//   새 goal     지금 작업을 멈추고 ABORTED("preempted by <새 id>") 로 끝낸 뒤 새 작업 시작 (선점)
//   cancel      트리를 halt (실행 중인 Nav2 goal 도 취소) 하고 CANCELED
//   트리 끝     SUCCESS → SUCCEEDED("done"), FAILURE → ABORTED("<실패한 노드> failed [at waypoint i/n]")
//
// 요청 처리와 tick 은 모두 step() 안에서 한다. 실행 노드가 step() 을 tick_hz 로 부른다.
class TaskServer
{
public:
  using ExecuteTask = bt_mission_msgs::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  // tree 는 blackboard 로 만든 task.xml 트리. 만들기 전에 prepareBlackboard() 를 불러 둬야 한다
  TaskServer(
    const rclcpp::Node::SharedPtr & node, BT::Tree & tree, BT::Blackboard::Ptr blackboard,
    bool log_transitions, const std::string & action_name = "execute_task");

  // 트리를 만들기 전에 부른다. waypoints 를 타입 없는 blackboard 항목으로 만들어 둔다
  // (Nav2 1.1.18 이하 NavigateThroughPoses 의 goals 포트 타입 문제, bt_navigator/README.md 3-3)
  static void prepareBlackboard(const BT::Blackboard::Ptr & blackboard);

  // 한 주기: 들어온 요청(goal, cancel)을 처리하고, 작업이 있으면 트리를 한 번 tick 한다
  void step();

  // 실행 중인 작업을 멈추고 ABORTED 로 끝낸다 (종료할 때)
  void stop(const std::string & reason);

  bool busy() const {return static_cast<bool>(active_);}

private:
  enum class Outcome {Succeeded, Aborted, Canceled};

  void start(const std::shared_ptr<GoalHandle> & goal);
  // 실행 중인 작업을 끝낸다. 트리를 halt 해서(실행 중인 Nav2 goal 취소) 다음 작업을 위해 초기화한다
  void finish(Outcome outcome, const std::string & message);
  void publishFeedback();

  rclcpp::Node::SharedPtr node_;
  BT::Tree & tree_;
  BT::Blackboard::Ptr blackboard_;
  TransitionLogger logger_;

  // 액션 서버 콜백은 이 노드만의 callback group 에 넣고 step() 에서 직접 처리한다
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp_action::Server<ExecuteTask>::SharedPtr server_;

  std::shared_ptr<GoalHandle> active_;   // 실행 중인 작업
  std::shared_ptr<GoalHandle> next_;     // 수락했지만 아직 시작하지 않은 작업 (다음 step 에서 시작)
  std::string task_id_;
  std::chrono::steady_clock::time_point started_;
  std::chrono::steady_clock::time_point last_feedback_;
  std::string last_running_;
  int last_reached_{0};
  int total_{0};                         // 이 작업의 점 수
  int count_{0};
  // 트리(루트)가 마지막으로 돌려준 상태. feedback 과 result 의 bt_status 로 보낸다
  BT::NodeStatus bt_status_{BT::NodeStatus::IDLE};
};

}  // namespace bt_mission

#endif  // BT_MISSION__TASK_SERVER_HPP_
