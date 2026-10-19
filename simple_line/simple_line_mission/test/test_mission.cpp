// mission.xml 의 흐름을 로봇 없이 확인한다.
// Nav2 노드(NavigateThroughPoses / NavigateToPose / Spin)만 가짜로 바꾸고, 나머지는 실제 노드를 쓴다.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "simple_line_mission/mission_nodes.hpp"
#include "tf2_ros/buffer.h"

using simple_line_mission::parsePoses;

TEST(ParsePoses, FillsMissingYawAlongTheRoute)
{
  const auto p = parsePoses("0.0,-1.0; 1.5,-1.0,90");
  ASSERT_EQ(p.size(), 2u);
  EXPECT_DOUBLE_EQ(p[0].x, 0.0);
  EXPECT_DOUBLE_EQ(p[0].y, -1.0);
  EXPECT_NEAR(p[0].yaw, 0.0, 1e-9);            // 다음 점 (1.5,-1) 쪽 = +x
  EXPECT_NEAR(p[1].yaw, M_PI / 2, 1e-9);       // 적은 값 90°

  const auto q = parsePoses(" 0,0 ; 0,2 ; ");   // 공백, 끝의 ';' 허용
  ASSERT_EQ(q.size(), 2u);
  EXPECT_NEAR(q[0].yaw, M_PI / 2, 1e-9);
  EXPECT_NEAR(q[1].yaw, M_PI / 2, 1e-9);       // 마지막 점: 이전 점에서 온 방향
}

TEST(ParsePoses, RejectsBadInput)
{
  EXPECT_THROW(parsePoses(""), std::invalid_argument);
  EXPECT_THROW(parsePoses("1.0"), std::invalid_argument);
  EXPECT_THROW(parsePoses("1.0,2.0,3.0,4.0"), std::invalid_argument);
  EXPECT_THROW(parsePoses("1.0,abc"), std::invalid_argument);
  EXPECT_THROW(parsePoses("1.0,2.0"), std::invalid_argument);   // 점 하나에 yaw 없음
}

// ---------------------------------------------------------------------------------------------

// 가짜 Nav2 노드: 3 tick 동안 RUNNING 후, 정해 둔 결과(기본 SUCCESS)를 돌려준다
std::map<std::string, std::deque<bool>> g_outcomes;            // 노드 이름 → 차례로 쓸 결과
std::map<std::string, std::vector<geometry_msgs::msg::PoseStamped>> g_goals;

class FakeNav2Action : public BT::StatefulActionNode
{
public:
  using BT::StatefulActionNode::StatefulActionNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort("goals"), BT::InputPort("goal"), BT::InputPort("behavior_tree"),
      BT::InputPort("spin_dist"), BT::InputPort("time_allowance"), BT::InputPort("is_recovery"),
    };
  }

  BT::NodeStatus onStart() override
  {
    std::vector<geometry_msgs::msg::PoseStamped> goals;
    geometry_msgs::msg::PoseStamped goal;
    if (getInput("goals", goals)) {
      g_goals[name()] = goals;
    } else if (getInput("goal", goal)) {
      g_goals[name()] = {goal};
    }
    ticks_ = 3;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (--ticks_ > 0) {
      return BT::NodeStatus::RUNNING;
    }
    auto & queue = g_outcomes[name()];
    bool ok = true;
    if (!queue.empty()) {
      ok = queue.front();
      queue.pop_front();
    }
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void onHalted() override {}

private:
  int ticks_{0};
};

// 노드 상태 변화를 "이름:상태" 로 모은다
class Recorder : public BT::StatusChangeLogger
{
public:
  explicit Recorder(BT::TreeNode * root)
  : BT::StatusChangeLogger(root) {}

  void callback(BT::Duration, const BT::TreeNode & node, BT::NodeStatus prev,
    BT::NodeStatus status) override
  {
    if (status != BT::NodeStatus::IDLE) {
      events.push_back(node.name() + ":" + BT::toStr(status));
    } else if (prev == BT::NodeStatus::RUNNING) {
      events.push_back(node.name() + ":HALTED");
    }
  }
  void flush() override {}

  int count(const std::string & event) const
  {
    return static_cast<int>(std::count(events.begin(), events.end(), event));
  }

  std::vector<std::string> events;
};

class MissionTree : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  BT::NodeStatus run()
  {
    BT::BehaviorTreeFactory factory;
    for (const char * id : {"NavigateThroughPoses", "NavigateToPose", "Spin"}) {
      factory.registerNodeType<FakeNav2Action>(id);
    }
    simple_line_mission::registerNodes(factory);

    auto node = std::make_shared<rclcpp::Node>("test_mission");
    auto blackboard = BT::Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", node);
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>(
      "tf_buffer", std::make_shared<tf2_ros::Buffer>(node->get_clock()));

    auto tree = factory.createTreeFromFile(MISSION_XML, blackboard);
    recorder = std::make_unique<Recorder>(tree.rootNode());
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING) {
      status = tree.tickRoot();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    recorder->setEnabled(false);
    return status;
  }

  void TearDown() override
  {
    g_outcomes.clear();
    g_goals.clear();
  }

  std::unique_ptr<Recorder> recorder;
};

TEST_F(MissionTree, AllStationsVisited)
{
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  const auto & r = *recorder;
  for (const char * e : {"go_a:SUCCESS", "inspect_a:SUCCESS", "go_b:SUCCESS", "look_around:SUCCESS",
      "report_b:SUCCESS", "go_home:SUCCESS", "done:SUCCESS"})
  {
    EXPECT_EQ(r.count(e), 1) << e;
  }
  // 주행 중 스캔은 go_b 가 끝나면 Parallel 이 멈춘다
  EXPECT_EQ(r.count("scan:RUNNING"), 1);
  EXPECT_EQ(r.count("scan:HALTED"), 1);
  EXPECT_EQ(r.count("skip_a:SUCCESS"), 0);
  EXPECT_EQ(r.count("skip_b:SUCCESS"), 0);

  // MakePoses → blackboard → NavigateThroughPoses 로 위치가 전달된다
  const auto & route_a = g_goals["go_a"];
  ASSERT_EQ(route_a.size(), 2u);
  EXPECT_DOUBLE_EQ(route_a[0].pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(route_a[1].pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(route_a[1].pose.position.y, -1.0);
  EXPECT_NEAR(route_a[1].pose.orientation.z, std::sin(M_PI / 4), 1e-9);   // yaw 90°
  EXPECT_EQ(route_a[1].header.frame_id, "map");
  ASSERT_EQ(g_goals["go_home"].size(), 1u);
  EXPECT_DOUBLE_EQ(g_goals["go_home"][0].pose.position.x, -2.0);
}

TEST_F(MissionTree, FailedStationIsSkipped)
{
  g_outcomes["go_a"] = {false};
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  const auto & r = *recorder;
  EXPECT_EQ(r.count("go_a:FAILURE"), 1);
  EXPECT_EQ(r.count("inspect_a:RUNNING"), 0);   // A 작업은 하지 않고
  EXPECT_EQ(r.count("skip_a:SUCCESS"), 1);      // 건너뛴 뒤
  EXPECT_EQ(r.count("go_b:SUCCESS"), 1);        // B 로 간다
  EXPECT_EQ(r.count("done:SUCCESS"), 1);
}

TEST_F(MissionTree, ReturnHomeIsRetriedOnce)
{
  g_outcomes["go_home"] = {false, true};
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(recorder->count("go_home:FAILURE"), 1);
  EXPECT_EQ(recorder->count("go_home:SUCCESS"), 1);
  EXPECT_EQ(recorder->count("done:SUCCESS"), 1);
}

TEST_F(MissionTree, MissionFailsWhenHomeIsUnreachable)
{
  g_outcomes["go_home"] = {false, false};
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(recorder->count("go_home:FAILURE"), 2);
  EXPECT_EQ(recorder->count("done:SUCCESS"), 0);
}
