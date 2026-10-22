// TaskServer (ExecuteTask 액션 서버 + task.xml) 를 로봇 없이 확인한다.
// 액션 클라이언트로 goal 을 보내고 결과, feedback, 선점, 취소를 본다.
// Nav2 노드(NavigateThroughPoses / NavigateToPose / Spin)만 가짜로 바꾸고, 나머지는 실제 노드를 쓴다.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "simple_line_mission/bt_common.hpp"
#include "simple_line_mission/mission_nodes.hpp"
#include "simple_line_mission/task_server.hpp"
#include "simple_line_mission_msgs/action/execute_task.hpp"
#include "test_fakes.hpp"
#include "tf2_ros/buffer.h"

using ExecuteTask = simple_line_mission_msgs::action::ExecuteTask;
using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteTask>;
using rclcpp_action::ResultCode;
using namespace std::chrono_literals;

class TaskServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    registerFakeNav2Nodes(factory_);
    simple_line_mission::registerNodes(factory_);

    node_ = std::make_shared<rclcpp::Node>("test_task_server");
    auto blackboard = simple_line_mission::createBlackboard(
      node_, std::make_shared<tf2_ros::Buffer>(node_->get_clock()), 100.0);
    simple_line_mission::TaskServer::prepareBlackboard(blackboard);
    tree_ = factory_.createTreeFromFile(TASK_XML, blackboard);
    recorder_ = std::make_unique<Recorder>(tree_.rootNode());
    server_ = std::make_unique<simple_line_mission::TaskServer>(node_, tree_, blackboard, false);

    // task_client.py 역할을 하는 쪽
    client_node_ = std::make_shared<rclcpp::Node>("test_task_client");
    client_ = rclcpp_action::create_client<ExecuteTask>(client_node_, "execute_task");
    ASSERT_TRUE(client_->wait_for_action_server(5s));
  }

  void TearDown() override
  {
    server_->stop("test end");
    server_.reset();
    g_outcomes.clear();
    g_goals.clear();
    g_ticks_to_finish = 3;
  }

  void step()
  {
    server_->step();
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(2ms);
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

  // goal 을 보내고 수락/거절 응답이 올 때까지 기다린다. 거절되면 nullptr
  GoalHandle::SharedPtr send(const ExecuteTask::Goal & goal)
  {
    rclcpp_action::Client<ExecuteTask>::SendGoalOptions options;
    options.feedback_callback =
      [this](GoalHandle::SharedPtr, const std::shared_ptr<const ExecuteTask::Feedback> feedback) {
        feedback_.push_back(*feedback);
      };
    auto future = client_->async_send_goal(goal, options);
    for (int i = 0; i < 1000 && future.wait_for(0s) != std::future_status::ready; ++i) {
      step();
    }
    return future.get();
  }

  GoalHandle::WrappedResult waitResult(const GoalHandle::SharedPtr & handle)
  {
    auto future = client_->async_get_result(handle);
    for (int i = 0; i < 3000 && future.wait_for(0s) != std::future_status::ready; ++i) {
      step();
    }
    EXPECT_EQ(future.wait_for(0s), std::future_status::ready);
    return future.get();
  }

  BT::BehaviorTreeFactory factory_;
  rclcpp::Node::SharedPtr node_;
  BT::Tree tree_;
  std::unique_ptr<Recorder> recorder_;
  std::unique_ptr<simple_line_mission::TaskServer> server_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<ExecuteTask>::SharedPtr client_;
  std::vector<ExecuteTask::Feedback> feedback_;
};

TEST_F(TaskServerTest, RunsATaskAndReportsFeedback)
{
  g_ticks_to_finish = 300;                        // 0.5초마다 오는 feedback 을 볼 수 있게 길게
  auto handle = send(makeGoal("t1", "inspect", {{0.0, -1.0}, {1.5, -1.0}}));
  ASSERT_TRUE(handle);
  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->success);
  EXPECT_EQ(result.result->message, "done");

  EXPECT_EQ(recorder_->count("go_inspect:SUCCESS"), 1);
  EXPECT_EQ(recorder_->count("work:SUCCESS"), 1);
  ASSERT_EQ(g_goals["go_inspect"].size(), 2u);          // goal 의 waypoints 가 그대로 goals 로
  EXPECT_DOUBLE_EQ(g_goals["go_inspect"][1].pose.position.x, 1.5);

  ASSERT_FALSE(feedback_.empty());
  EXPECT_EQ(feedback_.front().task_id, "t1");
  const auto saw = [this](const std::string & running) {
      return std::any_of(
        feedback_.begin(), feedback_.end(),
        [&](const ExecuteTask::Feedback & f) {return f.running == running;});
    };
  EXPECT_TRUE(saw("go_inspect"));
  EXPECT_TRUE(saw("work"));
}

TEST_F(TaskServerTest, RunsTasksOneAfterAnother)
{
  auto first = send(makeGoal("t1", "scan", {{1.5, 1.5}, {-1.0, 1.5}}));
  ASSERT_TRUE(first);
  EXPECT_EQ(waitResult(first).code, ResultCode::SUCCEEDED);
  auto second = send(makeGoal("t2", "look_around", {{-1.0, 1.5}}));
  ASSERT_TRUE(second);
  EXPECT_EQ(waitResult(second).code, ResultCode::SUCCEEDED);

  EXPECT_EQ(recorder_->count("go_scan:SUCCESS"), 1);
  EXPECT_EQ(recorder_->count("scanning:HALTED"), 1);   // 주행이 끝나면 스캔도 멈춤
  EXPECT_EQ(recorder_->count("go_look:SUCCESS"), 1);
  EXPECT_EQ(recorder_->count("spin:SUCCESS"), 1);
  EXPECT_FALSE(server_->busy());
}

TEST_F(TaskServerTest, NewGoalPreemptsTheRunningOne)
{
  g_ticks_to_finish = 100000;                     // a 는 끝나지 않는다
  auto a = send(makeGoal("a", "go", {{1.0, 0.0}}));
  ASSERT_TRUE(a);
  for (int i = 0; i < 20; ++i) {
    step();
  }
  g_ticks_to_finish = 3;                          // b 는 금방 끝난다
  auto b = send(makeGoal("b", "go", {{2.0, 0.0}}));
  ASSERT_TRUE(b);

  const auto result_a = waitResult(a);
  EXPECT_EQ(result_a.code, ResultCode::ABORTED);
  EXPECT_EQ(result_a.result->message, "preempted by b");
  EXPECT_EQ(waitResult(b).code, ResultCode::SUCCEEDED);
  EXPECT_EQ(recorder_->count("go_only:HALTED"), 1);     // a 의 이동을 멈춤 (실제로는 Nav2 goal 취소)
  EXPECT_DOUBLE_EQ(g_goals["go_only"][0].pose.position.x, 2.0);
}

TEST_F(TaskServerTest, CancelHaltsTheTask)
{
  g_ticks_to_finish = 100000;
  auto handle = send(makeGoal("c", "scan", {{1.0, 0.0}, {2.0, 0.0}}));
  ASSERT_TRUE(handle);
  for (int i = 0; i < 20; ++i) {
    step();
  }
  client_->async_cancel_goal(handle);
  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, ResultCode::CANCELED);
  EXPECT_EQ(result.result->message, "canceled");
  EXPECT_EQ(recorder_->count("go_scan:HALTED"), 1);
  EXPECT_EQ(recorder_->count("scanning:HALTED"), 1);
  EXPECT_FALSE(server_->busy());
}

TEST_F(TaskServerTest, FailedNavigationAbortsWithTheNodeName)
{
  g_outcomes["go_look"] = {false};
  auto handle = send(makeGoal("f", "look_around", {{1.0, 0.0}}));
  ASSERT_TRUE(handle);
  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, ResultCode::ABORTED);
  EXPECT_FALSE(result.result->success);
  EXPECT_EQ(result.result->message, "go_look failed");
  EXPECT_EQ(recorder_->count("spin:RUNNING"), 0);       // 이동이 실패하면 회전은 하지 않는다
}

TEST_F(TaskServerTest, RejectsATaskWithoutWaypoints)
{
  EXPECT_FALSE(send(makeGoal("r", "go", {})));
  EXPECT_FALSE(server_->busy());
}
