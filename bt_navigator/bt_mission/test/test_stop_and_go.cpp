// StopAndGo 와 ObstacleMode 를 로봇 없이 확인한다.
// 레이저 scan 은 테스트가 만들어 보내고, 로봇 위치는 TF 버퍼에 직접 넣는다. NavigateToPose 는 가짜다.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "bt_mission/bt_common.hpp"
#include "bt_mission/mission_nodes.hpp"
#include "std_msgs/msg/string.hpp"
#include "test_fakes.hpp"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using Obstacles = std::vector<std::array<double, 3>>;

namespace
{

// 로봇은 map 의 (0, 0) 에서 +x 를 본다. 레이저는 로봇 중심에 있다. goal 은 {goal} (기본 (3, 0))
const char * kTree = R"(
<root main_tree_to_execute="T">
  <BehaviorTree ID="T">
    <Sequence>
      <ObstacleMode mode="{obstacle}" avoid_bt="/trees/navigate_avoid.xml"
                    stop_and_go="{stop_and_go}" nav_bt="{nav_bt}"/>
      <StopAndGo name="stop_go" enabled="{stop_and_go}" goal="{goal}" blocked="{blocked}"
                 task_id="t1" resume_delay="0.3" scan_timeout="0.5" max_wait="{max_wait}">
        <NavigateToPose name="go" goal="{goal}" behavior_tree="{nav_bt}"/>
      </StopAndGo>
    </Sequence>
  </BehaviorTree>
</root>)";

geometry_msgs::msg::TransformStamped transform(
  const std::string & parent, const std::string & child, double x)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.rotation.w = 1.0;
  return t;
}

}  // namespace

class StopAndGoTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    registerFakeNav2Nodes(factory_);
    bt_mission::registerNodes(factory_);
    node_ = std::make_shared<rclcpp::Node>("test_stop_and_go");
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_->setTransform(transform("map", "base_link", 0.0), "test", true);
    tf_->setTransform(transform("base_link", "base_scan", 0.0), "test", true);
    blackboard_ = BT::Blackboard::create();
    blackboard_->set<rclcpp::Node::SharedPtr>("node", node_);
    blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);
    blackboard_->set("obstacle", std::string("stop"));
    blackboard_->set("max_wait", 0.0);
    setGoal(3.0);

    // 레이저 역할
    sensor_ = std::make_shared<rclcpp::Node>("test_laser");
    scan_pub_ = sensor_->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS());
    status_sub_ = sensor_->create_subscription<std_msgs::msg::String>(
      bt_mission::kStatusTopic, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {statuses_.push_back(msg->data);});
    g_ticks_to_finish = 60;               // 가짜 NavigateToPose 가 60 tick (0.3초) 동안 달린다
  }

  void TearDown() override {resetFakes();}

  void setGoal(double x)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = x;
    goal.pose.orientation.w = 1.0;
    blackboard_->set("goal", goal);
  }

  // 트리를 만들고, StopAndGo 의 scan 구독이 레이저와 연결될 때까지 기다린다.
  // obstacles 가 있으면 시작 전에 scan 을 한 번 보내 둔다
  void build(const Obstacles * first_scan = nullptr)
  {
    tree_ = factory_.createTreeFromText(kTree, blackboard_);
    recorder_ = std::make_unique<Recorder>(tree_.rootNode());
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline &&
      (scan_pub_->get_subscription_count() == 0 || status_sub_->get_publisher_count() == 0))
    {
      std::this_thread::sleep_for(10ms);
    }
    if (first_scan) {
      scan_pub_->publish(makeScan(*first_scan));
      std::this_thread::sleep_for(50ms);
    }
  }

  // seconds 동안 5ms 마다 tick 한다. obstacles 가 있으면 50ms 마다 그 scan 을 보낸다 (5Hz 레이저보다 촘촘히).
  // 트리가 끝나면 바로 돌아온다
  BT::NodeStatus run(double seconds, const Obstacles * obstacles)
  {
    auto status = BT::NodeStatus::RUNNING;
    const auto start = std::chrono::steady_clock::now();
    auto last_scan = start - 1s;
    while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(seconds)) {
      const auto now = std::chrono::steady_clock::now();
      if (obstacles && now - last_scan > 50ms) {
        scan_pub_->publish(makeScan(*obstacles));
        last_scan = now;
      }
      status = tree_.tickRoot();
      rclcpp::spin_some(sensor_);
      if (status != BT::NodeStatus::RUNNING) {
        break;
      }
      std::this_thread::sleep_for(5ms);
    }
    rclcpp::spin_some(sensor_);
    return status;
  }

  bool blocked() const
  {
    bool b = false;
    blackboard_->get("blocked", b);
    return b;
  }

  bool hasStatus(const std::string & prefix) const
  {
    return std::any_of(
      statuses_.begin(), statuses_.end(),
      [&](const std::string & s) {return s.rfind(prefix, 0) == 0;});
  }

  const Obstacles clear_{};
  BT::BehaviorTreeFactory factory_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
  std::unique_ptr<Recorder> recorder_;
  rclcpp::Node::SharedPtr sensor_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  std::vector<std::string> statuses_;
};

TEST_F(StopAndGoTest, ClearPathJustRunsTheChild)
{
  build(&clear_);
  EXPECT_EQ(run(5.0, &clear_), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(recorder_->count("go:RUNNING"), 1);
  EXPECT_FALSE(blocked());
  EXPECT_FALSE(hasStatus("t1 blocked"));
  EXPECT_EQ(g_behavior_trees["go"], "");          // stop: bt_navigator 의 기본 navigation BT
}

TEST_F(StopAndGoTest, StopsForAnObstacleAndGoesAgainWhenClear)
{
  const Obstacles ahead{{0.5, 0.0, 0.1}};          // 앞 0.4m 에 물체 (중심 0.5m, 반지름 0.1m)
  g_ticks_to_finish = 1000;
  build(&clear_);
  EXPECT_EQ(run(0.1, &clear_), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fakeGoals("go").size(), 1u);           // 출발

  EXPECT_EQ(run(0.3, &ahead), BT::NodeStatus::RUNNING);
  EXPECT_EQ(recorder_->count("go:HALTED"), 1);     // 이동을 멈췄다 (Nav2 goal 취소)
  EXPECT_TRUE(blocked());
  EXPECT_TRUE(hasStatus("t1 blocked: obstacle 0.40 m ahead"));

  g_ticks_to_finish = 3;
  EXPECT_EQ(run(0.15, &clear_), BT::NodeStatus::RUNNING);
  EXPECT_EQ(fakeGoals("go").size(), 1u);           // resume_delay(0.3초)가 지나기 전에는 기다린다
  EXPECT_EQ(run(5.0, &clear_), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(fakeGoals("go").size(), 2u);           // 같은 goal 을 다시 보냈다
  EXPECT_DOUBLE_EQ(fakeGoals("go")[1].pose.position.x, 3.0);
  EXPECT_FALSE(blocked());
  EXPECT_TRUE(hasStatus("t1 resumed after"));
}

TEST_F(StopAndGoTest, IgnoresWhatIsNotOnThePath)
{
  // 앞이지만 stop_distance(0.6m) 밖, 길 옆(half_width 0.25m 밖)
  const Obstacles off_path{{1.0, 0.0, 0.1}, {0.4, 0.45, 0.1}};
  build(&off_path);
  EXPECT_EQ(run(5.0, &off_path), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(hasStatus("t1 blocked"));
}

TEST_F(StopAndGoTest, IgnoresWhatIsBeyondTheGoal)
{
  setGoal(0.5);
  const Obstacles behind_goal{{0.7, 0.0, 0.1}};   // 앞 0.6m: stop_distance 안이지만 goal(0.5m) 너머
  build(&behind_goal);
  EXPECT_EQ(run(5.0, &behind_goal), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(hasStatus("t1 blocked"));
}

TEST_F(StopAndGoTest, NoScanCountsAsBlocked)
{
  build();
  EXPECT_EQ(run(0.3, nullptr), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(fakeGoals("go").empty());            // 레이저가 없으면 출발하지 않는다
  EXPECT_TRUE(blocked());
  EXPECT_TRUE(hasStatus("t1 blocked: no scan on scan"));
  EXPECT_EQ(run(5.0, &clear_), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(hasStatus("t1 resumed after"));
}

TEST_F(StopAndGoTest, GivesUpAfterMaxWait)
{
  const Obstacles ahead{{0.5, 0.0, 0.1}};
  blackboard_->set("max_wait", 0.3);
  build(&ahead);
  EXPECT_EQ(run(5.0, &ahead), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(fakeGoals("go").empty());
  EXPECT_TRUE(blocked());                          // 서버가 실패 이유로 쓴다
  EXPECT_TRUE(hasStatus("t1 gave up: blocked for 0.3 s"));
}

TEST_F(StopAndGoTest, AvoidAndNoneDoNotWatchTheScan)
{
  // 레이저를 보내지 않는다. stop 이면 막혔겠지만 avoid / none 은 보지 않고 바로 간다
  blackboard_->set("obstacle", std::string("avoid"));
  build();
  EXPECT_EQ(run(5.0, nullptr), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(g_behavior_trees["go"], "/trees/navigate_avoid.xml");   // bt_navigator 가 이 트리로 간다
  EXPECT_FALSE(hasStatus("t1 blocked"));

  blackboard_->set("obstacle", std::string("none"));
  tree_.haltTree();
  EXPECT_EQ(run(5.0, nullptr), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(g_behavior_trees["go"], "");
  EXPECT_FALSE(hasStatus("t1 blocked"));
}

TEST_F(StopAndGoTest, UnknownModeFails)
{
  blackboard_->set("obstacle", std::string("jump"));
  build(&clear_);
  EXPECT_EQ(run(1.0, &clear_), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(fakeGoals("go").empty());
}

TEST(FailureMessage, SaysWhenTheTaskGaveUpOnAnObstacle)
{
  EXPECT_EQ(bt_mission::failureMessage("go_only", 1, 3, false), "go_only failed at waypoint 2/3");
  EXPECT_EQ(
    bt_mission::failureMessage("", 1, 3, true), "blocked by an obstacle at waypoint 2/3");
}
