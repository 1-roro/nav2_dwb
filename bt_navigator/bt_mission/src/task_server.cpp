#include "bt_mission/task_server.hpp"

#include <memory>
#include <string>
#include <utility>

using namespace std::chrono_literals;

namespace bt_mission
{

TaskServer::TaskServer(
  const rclcpp::Node::SharedPtr & node, BT::Tree & tree, BT::Blackboard::Ptr blackboard,
  bool log_transitions, const std::string & action_name)
: node_(node), tree_(tree), blackboard_(std::move(blackboard)),
  logger_(tree.rootNode(), node->get_logger(), log_transitions)
{
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  server_ = rclcpp_action::create_server<ExecuteTask>(
    node_, action_name,
    // 새 goal: 점이 하나라도 있으면 받는다
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTask::Goal> goal) {
      if (goal->task.waypoints.empty()) {
        RCLCPP_WARN(
          node_->get_logger(), "[task] %s rejected: no waypoints", goal->task.id.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // cancel 요청: 받고, 실제 중단은 step() 에서
    [](const std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    // 수락된 goal: 다음 step() 에서 시작한다
    [this](const std::shared_ptr<GoalHandle> goal) {
      if (next_) {                       // 시작도 하기 전에 더 새 goal 이 왔다
        auto result = std::make_shared<ExecuteTask::Result>();
        result->message = "preempted before start";
        next_->abort(result);
      }
      next_ = goal;
    },
    rcl_action_server_get_default_options(), callback_group_);
}

void TaskServer::prepareBlackboard(const BT::Blackboard::Ptr & blackboard)
{
  blackboard->createEntry("waypoints", BT::PortInfo(BT::PortDirection::INOUT));
}

void TaskServer::step()
{
  // 지난 step 에서 수락한 goal 을 시작한다. 수락한 step 에서 바로 시작하면 첫 feedback 이 수락 응답보다
  // 먼저 도착해서, 클라이언트가 아직 모르는 goal 의 feedback 으로 버릴 수 있다.
  if (next_) {
    auto goal = next_;
    next_.reset();
    if (active_) {                       // 선점: 지금 작업을 멈추고 새 작업으로
      const auto & id = goal->get_goal()->task.id;
      finish(Outcome::Aborted, "preempted by " + (id.empty() ? std::string("a new task") : id));
    }
    start(goal);
  }
  executor_.spin_some();                 // goal, cancel 요청 처리
  if (!active_) {
    return;
  }
  if (active_->is_canceling()) {
    finish(Outcome::Canceled, "canceled");
    return;
  }

  BT::NodeStatus status;
  try {
    status = tree_.tickRoot();
  } catch (const std::exception & e) {   // 포트 타입 불일치 등 노드가 던진 예외
    finish(Outcome::Aborted, std::string("BT error: ") + e.what());
    return;
  }
  if (status == BT::NodeStatus::RUNNING) {
    publishFeedback();
  } else if (status == BT::NodeStatus::SUCCESS) {
    finish(Outcome::Succeeded, "done");
  } else {
    finish(
      Outcome::Aborted,
      failureMessage(
        logger_.lastFailed(), waypointsReached(blackboard_), total_, isBlocked(blackboard_)));
  }
}

void TaskServer::stop(const std::string & reason)
{
  if (next_) {
    auto result = std::make_shared<ExecuteTask::Result>();
    result->message = reason;
    next_->abort(result);
    next_.reset();
  }
  if (active_) {
    finish(Outcome::Aborted, reason);
  }
}

void TaskServer::start(const std::shared_ptr<GoalHandle> & goal)
{
  const auto & task = goal->get_goal()->task;
  // task.xml 의 {waypoints} {task_id} {action} {duration}
  task_id_ = writeTask(blackboard_, task, ++count_);
  total_ = static_cast<int>(task.waypoints.size());

  logger_.clear();
  last_running_.clear();
  last_reached_ = 0;
  last_blocked_ = false;
  started_ = std::chrono::steady_clock::now();
  last_feedback_ = started_;
  active_ = goal;
  RCLCPP_INFO(
    node_->get_logger(), "[task] %s started: %s, %zu poses", task_id_.c_str(),
    task.action.empty() ? "go" : task.action.c_str(), task.waypoints.size());
}

void TaskServer::finish(Outcome outcome, const std::string & message)
{
  tree_.haltTree();                      // 실행 중인 노드를 멈추고(Nav2 goal 취소) 초기화
  auto result = std::make_shared<ExecuteTask::Result>();
  result->success = outcome == Outcome::Succeeded;
  result->message = message;
  const char * state = "aborted";
  if (outcome == Outcome::Succeeded) {
    active_->succeed(result);
    state = "succeeded";
  } else if (outcome == Outcome::Canceled) {
    active_->canceled(result);
    state = "canceled";
  } else {
    active_->abort(result);
  }
  const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - started_;
  RCLCPP_INFO(
    node_->get_logger(), "[task] %s %s in %.1f s: %s", task_id_.c_str(), state,
    elapsed.count(), message.c_str());
  active_.reset();
}

void TaskServer::publishFeedback()
{
  // 실행 중인 노드, 도착한 점 수, 장애물로 멈춘 상태가 바뀌었을 때, 아니면 0.5초마다
  const auto now = std::chrono::steady_clock::now();
  const auto running = logger_.running();
  const int reached = waypointsReached(blackboard_);
  const bool blocked = isBlocked(blackboard_);
  if (running == last_running_ && reached == last_reached_ && blocked == last_blocked_ &&
    now - last_feedback_ < 500ms)
  {
    return;
  }
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  feedback->task_id = task_id_;
  feedback->running = running;
  feedback->waypoints_reached = reached;
  feedback->waypoints_total = total_;
  feedback->blocked = blocked;
  feedback->elapsed = std::chrono::duration<float>(now - started_).count();
  active_->publish_feedback(feedback);
  last_running_ = running;
  last_reached_ = reached;
  last_blocked_ = blocked;
  last_feedback_ = now;
}

}  // namespace bt_mission
