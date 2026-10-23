// TaskServerLifecycle (BtActionServer 버전) 을 test_task_server.cpp 와 같은 경우로 확인한다.
// Nav2 노드만 가짜(플러그인 test_fake_nav2)로 바꾸고, 이 패키지의 노드는 플러그인 bt_mission_plugin 으로 쓴다.
// BT 는 BtActionServer 의 실행 스레드에서 돌고, 서버 노드는 테스트의 다른 스레드가 spin 한다.
// TaskServer 와 결과가 다른 곳은 그렇다고 적어 두었다.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bt_mission/task_server_lifecycle.hpp"
#include "bt_mission_msgs/action/execute_task.hpp"
#include "test_fakes.hpp"

using ExecuteTask = bt_mission_msgs::action::ExecuteTask;
using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTask>;
using lifecycle_msgs::msg::State;
using rclcpp_action::ResultCode;
using namespace std::chrono_literals;

class TaskServerLifecycleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
      {"bt_xml", TASK_XML},
      {"plugin_lib_names", std::vector<std::string>{"test_fake_nav2", "bt_mission_plugin"}},
      {"log_transitions", false},
    });
    server_ = std::make_shared<bt_mission::TaskServerLifecycle>(options);
    executor_.add_node(server_->get_node_base_interface());
    spin_thread_ = std::thread([this]() {executor_.spin();});

    ASSERT_EQ(server_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(server_->activate().id(), State::PRIMARY_STATE_ACTIVE);

    client_node_ = std::make_shared<rclcpp::Node>("test_task_client");
    client_ = rclcpp_action::create_client<ExecuteTask>(client_node_, "execute_task");
    ASSERT_TRUE(client_->wait_for_action_server(5s));
    spinFor(100ms);                     // 서버도 클라이언트의 응답 구독을 찾을 시간 (test_task_server.cpp 참고)
  }

  void TearDown() override
  {
    if (server_->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
      server_->deactivate();
    }
    server_->cleanup();
    executor_.cancel();
    spin_thread_.join();
    executor_.remove_node(server_->get_node_base_interface());
    server_.reset();
    resetFakes();
  }

  static ExecuteTask::Goal makeGoal(
    const std::string & id, const std::string & action,
    const std::vector<std::pair<double, double>> & points)
  {
    ExecuteTask::Goal goal;
    goal.task.id = id;
    goal.task.action = action;
    goal.task.duration = 0.1;
    for (const auto & [x, y] : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation.w = 1.0;
      goal.task.waypoints.push_back(pose);
    }
    return goal;
  }

  // goal 을 보내고 수락/거절 응답을 기다린다. 거절되면 nullptr
  GoalHandle::SharedPtr send(const ExecuteTask::Goal & goal)
  {
    rclcpp_action::Client<ExecuteTask>::SendGoalOptions options;
    options.feedback_callback =
      [this](GoalHandle::SharedPtr, const std::shared_ptr<const ExecuteTask::Feedback> feedback) {
        feedback_.push_back(*feedback);
      };
    auto future = client_->async_send_goal(goal, options);
    if (rclcpp::spin_until_future_complete(client_node_, future, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      ADD_FAILURE() << "no response to goal " << goal.task.id;
      return nullptr;
    }
    return future.get();
  }

  GoalHandle::WrappedResult waitResult(const GoalHandle::SharedPtr & handle)
  {
    auto future = client_->async_get_result(handle);
    if (rclcpp::spin_until_future_complete(client_node_, future, 30s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      ADD_FAILURE() << "no result";
      GoalHandle::WrappedResult none;
      none.code = ResultCode::UNKNOWN;
      none.result = std::make_shared<ExecuteTask::Result>();
      return none;
    }
    return future.get();
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    rclcpp::spin_until_future_complete(
      client_node_, std::promise<void>().get_future(), duration);
  }

  bool sawFeedback(const std::string & running, int reached = -1) const
  {
    return std::any_of(
      feedback_.begin(), feedback_.end(),
      [&](const ExecuteTask::Feedback & f) {
        return f.running == running &&
        (reached < 0 || f.waypoints_reached == static_cast<unsigned>(reached));
      });
  }

  std::shared_ptr<bt_mission::TaskServerLifecycle> server_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<ExecuteTask>::SharedPtr client_;
  std::vector<ExecuteTask::Feedback> feedback_;
};

TEST_F(TaskServerLifecycleTest, RunsATaskAndReportsFeedback)
{
  g_ticks_to_finish = 300;                        // 10ms tick 으로 3초. 0.5초마다 오는 feedback 을 보려고
  auto handle = send(makeGoal("t1", "inspect", {{0.0, -1.0}, {1.5, -1.0}}));
  ASSERT_TRUE(handle);
  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->success);
  EXPECT_EQ(result.result->message, "done");

  EXPECT_EQ(countFakeEvent("go_inspect:SUCCESS"), 2);  // 점마다 NavigateToPose 한 번
  const auto goals = fakeGoals("go_inspect");
  ASSERT_EQ(goals.size(), 2u);                    // goal 의 점이 차례로 NavigateToPose 로
  EXPECT_DOUBLE_EQ(goals[1].pose.position.x, 1.5);

  ASSERT_FALSE(feedback_.empty());
  EXPECT_EQ(feedback_.front().task_id, "t1");
  EXPECT_EQ(feedback_.front().waypoints_total, 2u);
  EXPECT_TRUE(sawFeedback("go_inspect", 0));
  EXPECT_TRUE(sawFeedback("go_inspect", 1));
  EXPECT_TRUE(sawFeedback("work", 2));
}

TEST_F(TaskServerLifecycleTest, RunsTasksOneAfterAnother)
{
  auto first = send(makeGoal("t1", "scan", {{1.5, 1.5}, {-1.0, 1.5}}));
  ASSERT_TRUE(first);
  EXPECT_EQ(waitResult(first).code, ResultCode::SUCCEEDED);
  auto second = send(makeGoal("t2", "look_around", {{-1.0, 1.5}}));
  ASSERT_TRUE(second);
  EXPECT_EQ(waitResult(second).code, ResultCode::SUCCEEDED);

  EXPECT_EQ(countFakeEvent("go_scan:SUCCESS"), 2);
  EXPECT_EQ(countFakeEvent("go_look:SUCCESS"), 1);
  EXPECT_EQ(countFakeEvent("spin:SUCCESS"), 1);
}

TEST_F(TaskServerLifecycleTest, NewGoalPreemptsTheRunningOne)
{
  g_ticks_to_finish = 100000;                     // a 는 끝나지 않는다
  auto a = send(makeGoal("a", "go", {{1.0, 0.0}}));
  ASSERT_TRUE(a);
  spinFor(200ms);
  g_ticks_to_finish = 3;                          // b 는 금방 끝난다
  auto b = send(makeGoal("b", "go", {{2.0, 0.0}}));
  ASSERT_TRUE(b);

  const auto result_a = waitResult(a);
  EXPECT_EQ(result_a.code, ResultCode::ABORTED);
  EXPECT_EQ(result_a.result->message, "");        // TaskServer 는 "preempted by b". BtActionServer 는 빈 result
  EXPECT_EQ(waitResult(b).code, ResultCode::SUCCEEDED);
  EXPECT_EQ(countFakeEvent("go_only:HALTED"), 1); // a 의 이동을 멈춤 (실제로는 Nav2 goal 취소)
  const auto goals = fakeGoals("go_only");
  ASSERT_EQ(goals.size(), 2u);                    // a 의 점, b 의 점
  EXPECT_DOUBLE_EQ(goals[1].pose.position.x, 2.0);
}

TEST_F(TaskServerLifecycleTest, CancelHaltsTheTask)
{
  g_ticks_to_finish = 100000;
  auto handle = send(makeGoal("c", "scan", {{1.0, 0.0}, {2.0, 0.0}}));
  ASSERT_TRUE(handle);
  spinFor(200ms);
  EXPECT_TRUE(sawFeedback("go_scan, scanning"));
  client_->async_cancel_goal(handle);
  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, ResultCode::CANCELED);
  EXPECT_EQ(result.result->message, "canceled");
  EXPECT_EQ(countFakeEvent("go_scan:HALTED"), 1);
}

TEST_F(TaskServerLifecycleTest, FailedNavigationAbortsWithTheNodeName)
{
  {
    std::lock_guard<std::mutex> lock(g_fake_mutex);
    g_outcomes["go_look"] = {false};
  }
  auto handle = send(makeGoal("f", "look_around", {{1.0, 0.0}}));
  ASSERT_TRUE(handle);
  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, ResultCode::ABORTED);
  EXPECT_FALSE(result.result->success);
  EXPECT_EQ(result.result->message, "go_look failed at waypoint 1/1");
  EXPECT_EQ(countFakeEvent("spin:RUNNING"), 0);   // 이동이 실패하면 회전은 하지 않는다
}

TEST_F(TaskServerLifecycleTest, FailureStopsAtThatWaypoint)
{
  {
    std::lock_guard<std::mutex> lock(g_fake_mutex);
    g_outcomes["go_only"] = {true, false};        // 첫 점은 도착, 둘째 점에서 실패
  }
  auto handle = send(makeGoal("w", "go", {{1.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}}));
  ASSERT_TRUE(handle);
  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, ResultCode::ABORTED);
  EXPECT_EQ(result.result->message, "go_only failed at waypoint 2/3");
  EXPECT_EQ(fakeGoals("go_only").size(), 2u);     // 셋째 점으로는 가지 않는다
}

TEST_F(TaskServerLifecycleTest, TaskWithoutWaypointsIsAcceptedThenAborted)
{
  // TaskServer 는 수락 단계에서 거절(REJECTED)한다. BtActionServer 는 goal 을 먼저 수락하고
  // onGoalReceived 가 false 를 돌려주면 ABORTED 로 끝낸다
  auto handle = send(makeGoal("r", "go", {}));
  ASSERT_TRUE(handle);
  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, ResultCode::ABORTED);
  EXPECT_EQ(result.result->message, "");
}

TEST_F(TaskServerLifecycleTest, DeactivateStopsTheRunningTask)
{
  g_ticks_to_finish = 100000;
  auto handle = send(makeGoal("d", "go", {{1.0, 0.0}}));
  ASSERT_TRUE(handle);
  spinFor(200ms);
  auto result_future = client_->async_get_result(handle);
  EXPECT_EQ(server_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);   // Ctrl-C 도 이 경로
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node_, result_future, 5s),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, ResultCode::ABORTED);
  EXPECT_EQ(result.result->message, "task server deactivated");
  EXPECT_EQ(countFakeEvent("go_only:HALTED"), 1);

  // inactive 상태에서는 goal 을 거절한다
  EXPECT_FALSE(send(makeGoal("e", "go", {{1.0, 0.0}})));
}
