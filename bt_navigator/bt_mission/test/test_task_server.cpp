// TaskServer (ExecuteTask 액션 서버 + task.xml) 를 로봇 없이 확인한다.
// 액션 클라이언트로 goal 을 보내고 결과, feedback, 선점, 취소를 본다.
// Nav2 노드(NavigateThroughPoses / NavigateToPose / Spin)만 가짜로 바꾸고, 나머지는 실제 노드를 쓴다.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "bt_mission/bt_common.hpp"
#include "bt_mission/mission_nodes.hpp"
#include "bt_mission/task_server.hpp"
#include "bt_mission_msgs/action/execute_task.hpp"
#include "test_fakes.hpp"
#include "tf2_ros/buffer.h"

using ExecuteTask = bt_mission_msgs::action::ExecuteTask;
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
    bt_mission::registerNodes(factory_);

    node_ = std::make_shared<rclcpp::Node>("test_task_server");
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    auto blackboard = bt_mission::createBlackboard(node_, tf_, 100.0);
    bt_mission::TaskServer::prepareBlackboard(blackboard);
    tree_ = factory_.createTreeFromFile(TASK_XML, blackboard);
    recorder_ = std::make_unique<Recorder>(tree_.rootNode());
    server_ = std::make_unique<bt_mission::TaskServer>(node_, tree_, blackboard, false);

    // task_client.py 역할을 하는 쪽
    client_node_ = std::make_shared<rclcpp::Node>("test_task_client");
    client_ = rclcpp_action::create_client<ExecuteTask>(client_node_, "execute_task");
    ASSERT_TRUE(client_->wait_for_action_server(5s));
    // wait_for_action_server 는 클라이언트가 서버를 찾았다는 뜻일 뿐이다. 서버가 클라이언트의 응답 구독을
    // 아직 못 찾았으면 그 사이 보낸 응답은 사라지므로, 양쪽이 서로 찾을 시간을 조금 준다
    for (int i = 0; i < 50; ++i) {
      step();
    }
  }

  void TearDown() override
  {
    server_->stop("test end");
    server_.reset();
    resetFakes();
  }

  void step()
  {
    server_->step();
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(2ms);
  }

  // 이 테스트에는 레이저가 없으므로 obstacle 은 none (stop & go 는 test_stop_and_go.cpp 에서 본다)
  static ExecuteTask::Goal makeGoal(
    const std::string & id, const std::string & action,
    const std::vector<std::pair<double, double>> & points)
  {
    ExecuteTask::Goal goal;
    goal.task.id = id;
    goal.task.action = action;
    goal.task.duration = 0.1;
    goal.task.obstacle = bt_mission_msgs::msg::Task::OBSTACLE_NONE;
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
    if (future.wait_for(0s) != std::future_status::ready) {
      ADD_FAILURE() << "no response to goal " << goal.task.id;
      return nullptr;
    }
    return future.get();
  }

  GoalHandle::WrappedResult waitResult(const GoalHandle::SharedPtr & handle)
  {
    auto future = client_->async_get_result(handle);
    for (int i = 0; i < 3000 && future.wait_for(0s) != std::future_status::ready; ++i) {
      step();
    }
    if (future.wait_for(0s) != std::future_status::ready) {
      ADD_FAILURE() << "no result";
      GoalHandle::WrappedResult none;
      none.code = ResultCode::UNKNOWN;
      none.result = std::make_shared<ExecuteTask::Result>();
      return none;
    }
    return future.get();
  }

  BT::BehaviorTreeFactory factory_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  BT::Tree tree_;
  std::unique_ptr<Recorder> recorder_;
  std::unique_ptr<bt_mission::TaskServer> server_;
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

  EXPECT_EQ(recorder_->count("go_inspect:SUCCESS"), 2);  // 점마다 NavigateToPose 한 번
  EXPECT_EQ(recorder_->count("work:SUCCESS"), 1);
  ASSERT_EQ(g_goals["go_inspect"].size(), 2u);          // goal 의 점이 차례로 NavigateToPose 로
  EXPECT_DOUBLE_EQ(g_goals["go_inspect"][0].pose.position.y, -1.0);
  EXPECT_DOUBLE_EQ(g_goals["go_inspect"][1].pose.position.x, 1.5);

  ASSERT_FALSE(feedback_.empty());
  EXPECT_EQ(feedback_.front().task_id, "t1");
  EXPECT_EQ(feedback_.front().waypoints_total, 2u);
  const auto saw = [this](const std::string & running, unsigned reached) {
      return std::any_of(
        feedback_.begin(), feedback_.end(),
        [&](const ExecuteTask::Feedback & f) {
          return f.running == running && f.waypoints_reached == reached;
        });
    };
  EXPECT_TRUE(saw("go_inspect", 0));              // 첫 점으로 가는 중
  EXPECT_TRUE(saw("go_inspect", 1));              // 첫 점 도착, 둘째 점으로 가는 중
  EXPECT_TRUE(saw("work", 2));                    // 마지막 점 도착 후 작업
  // 점 사이에도 실행 중인 노드가 비지 않는다 (도착한 tick 에 다음 점을 바로 시작)
  for (const auto & f : feedback_) {
    EXPECT_FALSE(f.running.empty());
  }
}

TEST_F(TaskServerTest, RunsTasksOneAfterAnother)
{
  auto first = send(makeGoal("t1", "scan", {{1.5, 1.5}, {-1.0, 1.5}}));
  ASSERT_TRUE(first);
  EXPECT_EQ(waitResult(first).code, ResultCode::SUCCEEDED);
  auto second = send(makeGoal("t2", "look_around", {{-1.0, 1.5}}));
  ASSERT_TRUE(second);
  EXPECT_EQ(waitResult(second).code, ResultCode::SUCCEEDED);

  EXPECT_EQ(recorder_->count("go_scan:SUCCESS"), 2);
  EXPECT_EQ(recorder_->count("scanning:HALTED"), 1);   // 마지막 점에 도착하면 스캔도 멈춤
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
  ASSERT_EQ(g_goals["go_only"].size(), 2u);             // a 의 점, b 의 점
  EXPECT_DOUBLE_EQ(g_goals["go_only"][1].pose.position.x, 2.0);
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
  EXPECT_EQ(result.result->message, "go_look failed at waypoint 1/1");
  EXPECT_EQ(recorder_->count("spin:RUNNING"), 0);       // 이동이 실패하면 회전은 하지 않는다
}

TEST_F(TaskServerTest, FailureStopsAtThatWaypoint)
{
  g_outcomes["go_only"] = {true, false};                // 첫 점은 도착, 둘째 점에서 실패
  auto handle = send(makeGoal("w", "go", {{1.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}}));
  ASSERT_TRUE(handle);
  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, ResultCode::ABORTED);
  EXPECT_EQ(result.result->message, "go_only failed at waypoint 2/3");
  EXPECT_EQ(g_goals["go_only"].size(), 2u);             // 셋째 점으로는 가지 않는다
}

TEST_F(TaskServerTest, StopAndGoWaitsForTheObstacleAndReportsIt)
{
  // 로봇은 map 의 (0, 0) 에서 +x 를 보고, 레이저는 로봇 중심에 있다
  geometry_msgs::msg::TransformStamped t;
  t.transform.rotation.w = 1.0;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_->setTransform(t, "test", true);
  t.header.frame_id = "base_link";
  t.child_frame_id = "base_scan";
  tf_->setTransform(t, "test", true);
  auto scan_pub = client_node_->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS());
  // steps 만큼 step 하면서 20 step(약 50ms) 마다 scan 을 보낸다. until 이 참이 되면 멈춘다
  const auto run = [&](int steps, const std::vector<std::array<double, 3>> & obstacles,
      const std::function<bool()> & until) {
      for (int i = 0; i < steps && !until(); ++i) {
        if (i % 20 == 0) {
          scan_pub->publish(makeScan(obstacles));
        }
        step();
      }
      return until();
    };
  const auto saw_blocked = [this](bool value) {
      return std::any_of(
        feedback_.begin(), feedback_.end(),
        [value](const ExecuteTask::Feedback & f) {return f.blocked == value;});
    };

  g_ticks_to_finish = 100000;
  auto goal = makeGoal("s", "go", {{3.0, 0.0}});
  goal.task.obstacle = "stop";
  auto handle = send(goal);
  ASSERT_TRUE(handle);
  ASSERT_TRUE(run(1000, {}, [] {return fakeGoals("go_only").size() == 1;}));   // 출발

  // 앞 0.4m 에 물체 → 이동을 멈추고(Nav2 goal 취소) feedback 의 blocked 가 true
  feedback_.clear();
  ASSERT_TRUE(run(1000, {{0.5, 0.0, 0.1}}, [&] {return saw_blocked(true);}));
  EXPECT_EQ(countFakeEvent("go_only:HALTED"), 1);
  EXPECT_TRUE(feedback_.back().running.empty());        // 기다리는 동안 RUNNING 인 이동 노드가 없다

  // 길이 비면 1초(resume_delay) 뒤 같은 점으로 다시 출발하고, blocked 가 false
  g_ticks_to_finish = 3;
  feedback_.clear();
  ASSERT_TRUE(run(2000, {}, [] {return fakeGoals("go_only").size() == 2;}));
  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, ResultCode::SUCCEEDED);
  EXPECT_TRUE(saw_blocked(false));
}

TEST_F(TaskServerTest, WithoutAScanTheRobotDoesNotStart)
{
  // 레이저가 없으면 막힌 것으로 보고 출발하지 않는다. task.xml 의 max_wait 는 0(계속 기다림)이라 취소로 끝낸다
  auto goal = makeGoal("n", "go", {{1.0, 0.0}});
  goal.task.obstacle = "stop";
  auto handle = send(goal);
  ASSERT_TRUE(handle);
  for (int i = 0; i < 1000 && feedback_.empty(); ++i) {
    step();
  }
  ASSERT_FALSE(feedback_.empty());
  EXPECT_TRUE(feedback_.back().blocked);
  EXPECT_TRUE(fakeGoals("go_only").empty());            // 출발하지 않았다
  client_->async_cancel_goal(handle);
  EXPECT_EQ(waitResult(handle).code, ResultCode::CANCELED);
}

TEST_F(TaskServerTest, RejectsATaskWithoutWaypoints)
{
  EXPECT_FALSE(send(makeGoal("r", "go", {})));
  EXPECT_FALSE(server_->busy());
}
