#ifndef BT_MISSION__TASK_SERVER_LIFECYCLE_HPP_
#define BT_MISSION__TASK_SERVER_LIFECYCLE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "nav2_behavior_tree/bt_action_server.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "bt_mission/bt_common.hpp"
#include "bt_mission_msgs/action/execute_task.hpp"

namespace bt_mission
{

// TaskServer 와 같은 일을 Nav2 의 nav2_behavior_tree::BtActionServer 로 하는 lifecycle 노드.
// bt_navigator 안의 navigator (NavigateToPoseNavigator 등)와 같은 구조다.
//
//   BtActionServer 가 하는 일    액션 서버, 플러그인 이름으로 노드 등록, activate 때 트리 만들기,
//                                별도 스레드에서 tick 루프, cancel → halt, 끝나면 haltAllActions, 결과 보내기,
//                                BT 상태 변화를 behavior_tree_log 토픽으로
//   이 클래스가 채우는 콜백       onGoalReceived  goal 확인, blackboard 채우기
//                                onLoop          feedback
//                                onPreempt       새 goal 이 오면 지금 작업을 멈추고 새 작업으로
//                                onCompletion    result 의 success / message
//
// configure → activate 가 되어야 goal 을 받는다 (lifecycle_manager, 또는 ros2 lifecycle set).
// 콜백은 모두 BtActionServer 의 실행 스레드에서 불린다.
class TaskServerLifecycle : public nav2_util::LifecycleNode
{
public:
  using ExecuteTask = bt_mission_msgs::action::ExecuteTask;

  explicit TaskServerLifecycle(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  bool onGoalReceived(ExecuteTask::Goal::ConstSharedPtr goal);
  void onLoop();
  void onPreempt(ExecuteTask::Goal::ConstSharedPtr goal);
  void onCompletion(ExecuteTask::Result::SharedPtr result, nav2_behavior_tree::BtStatus status);

  // goal 의 작업을 blackboard 에 쓰고 진행 상태를 초기화한다
  void startTask(const ExecuteTask::Goal & goal);
  double elapsed() const;

  std::unique_ptr<nav2_behavior_tree::BtActionServer<ExecuteTask>> bt_server_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<TransitionLogger> logger_;
  bool log_transitions_{true};
  std::atomic<bool> deactivating_{false};

  std::string task_id_;
  std::chrono::steady_clock::time_point started_;
  std::chrono::steady_clock::time_point last_feedback_;
  std::string last_running_;
  int last_reached_{0};
  int total_{0};                         // 이 작업의 점 수
  int count_{0};
};

}  // namespace bt_mission

#endif  // BT_MISSION__TASK_SERVER_LIFECYCLE_HPP_
