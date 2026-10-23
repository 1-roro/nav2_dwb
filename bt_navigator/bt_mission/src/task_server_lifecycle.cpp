#include "bt_mission/task_server_lifecycle.hpp"

#include <memory>
#include <string>
#include <vector>

#include "bt_mission/task_server.hpp"

using namespace std::chrono_literals;
using nav2_behavior_tree::BtStatus;
using nav2_util::CallbackReturn;

namespace bt_mission
{

TaskServerLifecycle::TaskServerLifecycle(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("task_server_lifecycle", "", options)
{
  // BtActionServer 는 이 플러그인 이름들로만 BT 노드를 등록한다 (bt_navigator 의 plugin_lib_names 와 같다).
  // 직접 만든 노드도 플러그인(bt_mission_plugin)으로 넣는다
  auto plugins = nav2BtPlugins();
  plugins.push_back("bt_mission_plugin");
  declare_parameter("plugin_lib_names", plugins);
  declare_parameter("bt_xml", std::string("task.xml"));
  declare_parameter("log_transitions", true);
  // BtActionServer 가 만드는 파라미터. 기본값만 task_server 와 맞춘다 (BtActionServer 기본은 20 / 1000 ms)
  declare_parameter("default_server_timeout", 1000);
  declare_parameter("wait_for_service_timeout", 5000);
}

CallbackReturn TaskServerLifecycle::on_configure(const rclcpp_lifecycle::State &)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);
  log_transitions_ = get_parameter("log_transitions").as_bool();

  using std::placeholders::_1;
  using std::placeholders::_2;
  bt_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ExecuteTask>>(
    shared_from_this(), "execute_task",
    get_parameter("plugin_lib_names").as_string_array(),
    resolveTreePath(get_parameter("bt_xml").as_string()),
    std::bind(&TaskServerLifecycle::onGoalReceived, this, _1),
    std::bind(&TaskServerLifecycle::onLoop, this),
    std::bind(&TaskServerLifecycle::onPreempt, this, _1),
    std::bind(&TaskServerLifecycle::onCompletion, this, _1, _2));
  if (!bt_server_->on_configure()) {
    return CallbackReturn::FAILURE;
  }
  // BtActionServer 가 node, server_timeout 등을 넣은 blackboard 에 나머지를 더한다
  auto blackboard = bt_server_->getBlackboard();
  blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_);
  TaskServer::prepareBlackboard(blackboard);
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskServerLifecycle::on_activate(const rclcpp_lifecycle::State &)
{
  // 여기서 트리를 만든다. Nav2 BT 노드는 만들어질 때 Nav2 액션 서버를 찾는다 (wait_for_service_timeout)
  if (!bt_server_->on_activate()) {
    RCLCPP_ERROR(
      get_logger(), "failed to create the tree. Is Nav2 (bt_navigator, behavior_server) up?");
    return CallbackReturn::FAILURE;
  }
  logger_ = std::make_unique<TransitionLogger>(
    bt_server_->getTree().rootNode(), get_logger(), log_transitions_);
  createBond();                          // lifecycle_manager 가 이 노드가 살아 있는지 본다
  RCLCPP_INFO(
    get_logger(), "ready: action /execute_task, tree %s",
    get_parameter("bt_xml").as_string().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskServerLifecycle::on_deactivate(const rclcpp_lifecycle::State &)
{
  // 실행 중인 작업이 있으면 BtActionServer 가 트리를 halt(Nav2 goal 취소)하고 끝날 때까지 기다린다
  deactivating_ = true;
  bt_server_->on_deactivate();
  deactivating_ = false;
  destroyBond();
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskServerLifecycle::on_cleanup(const rclcpp_lifecycle::State &)
{
  logger_.reset();
  if (bt_server_) {
    bt_server_->on_cleanup();
    bt_server_.reset();
  }
  tf_listener_.reset();
  tf_buffer_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskServerLifecycle::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

bool TaskServerLifecycle::onGoalReceived(ExecuteTask::Goal::ConstSharedPtr goal)
{
  // false 를 돌려주면 BtActionServer 가 goal 을 ABORTED 로 끝낸다 (이미 수락된 뒤라 거절은 아니다)
  if (goal->task.waypoints.empty()) {
    RCLCPP_WARN(get_logger(), "[task] %s rejected: no waypoints", goal->task.id.c_str());
    return false;
  }
  startTask(*goal);
  return true;
}

void TaskServerLifecycle::onLoop()
{
  // tick 한 번마다 불린다. 트리가 끝난 tick 에도 불리므로, RUNNING 일 때만
  // 실행 중인 노드가 바뀌었을 때, 아니면 0.5초마다 feedback
  if (bt_server_->getTree().rootNode()->status() != BT::NodeStatus::RUNNING) {
    return;
  }
  // BtActionServer 는 goal 을 수락하자마자 다른 스레드에서 tick 을 시작한다. 그대로 보내면 첫 feedback 이
  // 수락 응답보다 먼저 도착해서, 클라이언트가 모르는 goal 의 것으로 버리는 일이 있다
  // (gtest 를 10번 돌렸을 때 8번). 그래서 시작 후 50ms 동안은 보내지 않는다
  const auto now = std::chrono::steady_clock::now();
  if (now - started_ < 50ms) {
    return;
  }
  const auto running = logger_->running();
  const int reached = waypointsReached(bt_server_->getBlackboard());
  if (running == last_running_ && reached == last_reached_ && now - last_feedback_ < 500ms) {
    return;
  }
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->task_id = task_id_;
  feedback->running = running;
  feedback->waypoints_reached = reached;
  feedback->waypoints_total = total_;
  feedback->elapsed = static_cast<float>(elapsed());
  bt_server_->publishFeedback(feedback);
  last_running_ = running;
  last_reached_ = reached;
  last_feedback_ = now;
}

void TaskServerLifecycle::onPreempt(ExecuteTask::Goal::ConstSharedPtr goal)
{
  // 트리가 이번 tick 에 막 끝났으면 지금 작업의 결과를 먼저 보낸다. 새 goal 은 BtActionServer 가
  // 이어서 시작한다 (onGoalReceived 부터)
  if (bt_server_->getTree().rootNode()->status() != BT::NodeStatus::RUNNING) {
    return;
  }
  if (goal->task.waypoints.empty()) {
    RCLCPP_WARN(get_logger(), "[task] %s rejected: no waypoints", goal->task.id.c_str());
    bt_server_->terminatePendingGoal();
    return;
  }
  RCLCPP_INFO(
    get_logger(), "[task] %s aborted in %.1f s: preempted by %s", task_id_.c_str(), elapsed(),
    goal->task.id.c_str());
  bt_server_->haltTree();                // 지금 작업의 노드를 멈춘다 (Nav2 goal 취소). 다음 tick 은 처음부터
  // 기다리던 goal 을 지금 goal 로 바꾼다. 이전 goal 은 ABORTED 가 되는데, result 는 빈 값이다
  startTask(*bt_server_->acceptPendingGoal());
}

void TaskServerLifecycle::onCompletion(ExecuteTask::Result::SharedPtr result, BtStatus status)
{
  // BtActionServer 가 이 result 로 SUCCEEDED / ABORTED / CANCELED 를 보낸다
  result->success = status == BtStatus::SUCCEEDED;
  const char * state = "aborted";
  if (status == BtStatus::SUCCEEDED) {
    result->message = "done";
    state = "succeeded";
  } else if (status == BtStatus::CANCELED) {
    // cancel 요청, 또는 deactivate (Ctrl-C 도 여기로 온다)
    result->message = deactivating_ ? "task server deactivated" : "canceled";
    state = deactivating_ ? "stopped" : "canceled";
  } else {
    result->message = failureMessage(
      logger_->lastFailed(), waypointsReached(bt_server_->getBlackboard()), total_);
  }
  RCLCPP_INFO(
    get_logger(), "[task] %s %s in %.1f s: %s", task_id_.c_str(), state, elapsed(),
    result->message.c_str());
}

void TaskServerLifecycle::startTask(const ExecuteTask::Goal & goal)
{
  task_id_ = writeTask(bt_server_->getBlackboard(), goal.task, ++count_);
  total_ = static_cast<int>(goal.task.waypoints.size());
  logger_->clear();
  last_running_.clear();
  last_reached_ = 0;
  started_ = std::chrono::steady_clock::now();
  last_feedback_ = started_;
  RCLCPP_INFO(
    get_logger(), "[task] %s started: %s, %zu poses", task_id_.c_str(),
    goal.task.action.empty() ? "go" : goal.task.action.c_str(), goal.task.waypoints.size());
}

double TaskServerLifecycle::elapsed() const
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
}

}  // namespace bt_mission
