// task_loop.xml 을 로봇 없이 확인한다.
// mission/task 로 Task 를 보내고, BT 가 action 에 맞는 노드를 실행하는지와 mission/status 보고를 본다.
// Nav2 노드(NavigateThroughPoses / NavigateToPose / Spin)만 가짜로 바꾸고, 나머지는 실제 노드를 쓴다.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "bt_mission/mission_nodes.hpp"
#include "bt_mission_msgs/msg/task.hpp"
#include "std_msgs/msg/string.hpp"
#include "test_fakes.hpp"
#include "tf2_ros/buffer.h"

using bt_mission_msgs::msg::Task;
using namespace std::chrono_literals;

class TaskLoopTree : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    registerFakeNav2Nodes(factory_);
    bt_mission::registerNodes(factory_);

    node_ = std::make_shared<rclcpp::Node>("test_task_loop");
    auto blackboard = BT::Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>(
      "tf_buffer", std::make_shared<tf2_ros::Buffer>(node_->get_clock()));
    tree_ = factory_.createTreeFromFile(TASK_LOOP_XML, blackboard);
    recorder_ = std::make_unique<Recorder>(tree_.rootNode());

    // task_sender.py 역할을 하는 쪽
    client_ = std::make_shared<rclcpp::Node>("test_task_client");
    task_pub_ = client_->create_publisher<Task>("mission/task", rclcpp::QoS(10));
    status_sub_ = client_->create_subscription<std_msgs::msg::String>(
      bt_mission::kStatusTopic, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {statuses_.push_back(msg->data);});
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline &&
      (task_pub_->get_subscription_count() == 0 || status_sub_->get_publisher_count() == 0))
    {
      std::this_thread::sleep_for(10ms);
    }
  }

  void TearDown() override
  {
    tree_.haltTree();
    g_outcomes.clear();
    g_goals.clear();
  }

  // 이 테스트에는 레이저가 없으므로 obstacle 은 none (stop & go 는 test_stop_and_go.cpp 에서 본다)
  static Task makeTask(
    const std::string & id, const std::string & action,
    const std::vector<std::pair<double, double>> & points, double duration = 0.1)
  {
    Task task;
    task.id = id;
    task.action = action;
    task.duration = duration;
    task.obstacle = Task::OBSTACLE_NONE;
    for (const auto & [x, y] : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation.w = 1.0;
      task.waypoints.push_back(pose);
    }
    return task;
  }

  // mission/status 에 text 가 올 때까지 tick 한다
  bool tickUntil(const std::string & text, std::chrono::seconds timeout = 10s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      tree_.tickRoot();
      rclcpp::spin_some(client_);
      if (has(text)) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  bool has(const std::string & text) const
  {
    return std::find(statuses_.begin(), statuses_.end(), text) != statuses_.end();
  }

  int indexOf(const std::string & text) const
  {
    return static_cast<int>(std::find(statuses_.begin(), statuses_.end(), text) - statuses_.begin());
  }

  BT::BehaviorTreeFactory factory_;
  rclcpp::Node::SharedPtr node_;
  BT::Tree tree_;
  std::unique_ptr<Recorder> recorder_;
  rclcpp::Node::SharedPtr client_;
  rclcpp::Publisher<Task>::SharedPtr task_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  std::vector<std::string> statuses_;
};

TEST_F(TaskLoopTree, ActionChoosesTheBehavior)
{
  task_pub_->publish(makeTask("t1", "inspect", {{0.0, -1.0}, {1.5, -1.0}}));
  task_pub_->publish(makeTask("t2", "scan", {{1.5, 1.5}, {-1.0, 1.5}}));
  task_pub_->publish(makeTask("t3", "look_around", {{-1.0, 1.5}}));
  task_pub_->publish(makeTask("t4", "", {{-2.0, -1.0}}));        // action 이 비면 이동만
  task_pub_->publish(makeTask("t5", "dance", {{0.0, 0.0}}));     // 모르는 action 도 이동만
  ASSERT_TRUE(tickUntil("t5 done (SUCCESS)"));

  // 받은 순서대로 하나씩 실행하고 보고한다
  EXPECT_TRUE(has("t1 accepted: inspect, 2 poses, obstacle none"));
  for (const char * id : {"t1", "t2", "t3", "t4", "t5"}) {
    EXPECT_LT(
      indexOf(std::string(id) + " started (RUNNING)"),
      indexOf(std::string(id) + " done (SUCCESS)")) << id;
  }
  EXPECT_LT(indexOf("t1 done (SUCCESS)"), indexOf("t2 started (RUNNING)"));
  EXPECT_LT(indexOf("t4 done (SUCCESS)"), indexOf("t5 started (RUNNING)"));

  // 점마다 도착을 알린 뒤 작업을 끝낸다
  EXPECT_LT(indexOf("t1 arrived 1/2"), indexOf("t1 arrived 2/2"));
  EXPECT_LT(indexOf("t1 arrived 2/2"), indexOf("t1 done (SUCCESS)"));
  EXPECT_TRUE(has("t2 arrived 2/2"));

  // action 별로 다른 가지가 실행된다. 이동 노드는 점마다 한 번씩 SUCCESS
  const auto & r = *recorder_;
  EXPECT_EQ(r.count("go_inspect:SUCCESS"), 2);
  EXPECT_EQ(r.count("work:SUCCESS"), 1);            // inspect: 마지막 점에 도착한 뒤 작업
  EXPECT_EQ(r.count("go_scan:SUCCESS"), 2);
  EXPECT_EQ(r.count("scanning:HALTED"), 1);         // scan: 주행이 끝나면 스캔도 멈춤
  EXPECT_EQ(r.count("go_look:SUCCESS"), 1);
  EXPECT_EQ(r.count("spin:SUCCESS"), 1);            // look_around: 도착 후 회전
  EXPECT_EQ(r.count("go_only:SUCCESS"), 2);         // t4, t5
  EXPECT_EQ(r.count("report_failed:SUCCESS"), 0);

  // Task 의 점이 차례로 NavigateToPose 의 goal 로 간다
  ASSERT_EQ(g_goals["go_inspect"].size(), 2u);
  EXPECT_DOUBLE_EQ(g_goals["go_inspect"][0].pose.position.y, -1.0);
  EXPECT_DOUBLE_EQ(g_goals["go_inspect"][1].pose.position.x, 1.5);
  EXPECT_EQ(g_goals["go_inspect"][1].header.frame_id, "map");

  // 작업은 마지막 점에 도착한 다음에 시작한다
  const auto & e = r.events;
  const auto last_arrival = std::find(e.rbegin(), e.rend(), "go_inspect:SUCCESS");
  const auto work_start = std::find(e.begin(), e.end(), "work:RUNNING");
  EXPECT_LT(e.rend() - last_arrival - 1, work_start - e.begin());
}

TEST_F(TaskLoopTree, FailureStopsAtThatWaypoint)
{
  g_outcomes["go_only"] = {true, false};          // 첫 점은 도착, 둘째 점에서 실패
  task_pub_->publish(makeTask("t1", "go", {{1.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}}));
  ASSERT_TRUE(tickUntil("t1 failed (FAILURE)"));
  EXPECT_TRUE(has("t1 arrived 1/3"));
  EXPECT_FALSE(has("t1 arrived 2/3"));
  EXPECT_EQ(g_goals["go_only"].size(), 2u);       // 셋째 점으로는 가지 않는다
}

TEST_F(TaskLoopTree, FailedTaskIsReportedAndTheLoopGoesOn)
{
  g_outcomes["go_only"] = {false, true};
  task_pub_->publish(makeTask("t1", "go", {{1.0, 0.0}}));
  task_pub_->publish(makeTask("t2", "go", {{2.0, 0.0}}));
  ASSERT_TRUE(tickUntil("t2 done (SUCCESS)"));
  EXPECT_TRUE(has("t1 failed (FAILURE)"));
  EXPECT_FALSE(has("t1 done (SUCCESS)"));
}

TEST_F(TaskLoopTree, TaskWithoutWaypointsIsRejected)
{
  task_pub_->publish(makeTask("t1", "inspect", {}));
  task_pub_->publish(makeTask("", "go", {{1.0, 0.0}}));       // id 가 없으면 BT 가 붙인다
  ASSERT_TRUE(tickUntil("task2 done (SUCCESS)"));
  EXPECT_TRUE(has("t1 rejected: no waypoints"));
  EXPECT_FALSE(has("t1 started (RUNNING)"));
  EXPECT_EQ(recorder_->count("work:RUNNING"), 0);
}

TEST_F(TaskLoopTree, WaitsWhileThereIsNoTask)
{
  for (int i = 0; i < 50; ++i) {
    EXPECT_EQ(tree_.tickRoot(), BT::NodeStatus::RUNNING);
    rclcpp::spin_some(client_);
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(has("waiting for a task"));
  EXPECT_EQ(recorder_->count("go_only:RUNNING"), 0);
}
