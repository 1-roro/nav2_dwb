// 테스트가 같이 쓰는 가짜 Nav2 노드와 상태 기록기
//
// 가짜 노드의 상태(g_outcomes 등)는 test_fakes.cpp (라이브러리 test_fake_nav2) 에 하나만 둔다.
// test_task_server_lifecycle 에서는 BtActionServer 가 이 라이브러리를 플러그인 이름으로 불러오는데,
// 테스트 실행 파일이 이미 링크한 라이브러리와 같은 것이라 테스트와 가짜 노드가 같은 상태를 본다.

#ifndef TEST_FAKES_HPP_
#define TEST_FAKES_HPP_

#include <algorithm>
#include <atomic>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

// 가짜 Nav2 노드: g_ticks_to_finish tick 동안 RUNNING 후, 정해 둔 결과(기본 SUCCESS)를 돌려준다
extern std::map<std::string, std::deque<bool>> g_outcomes;            // 노드 이름 → 차례로 쓸 결과
// 노드 이름 → 받은 목표. goals(NavigateThroughPoses)는 받은 목록, goal(NavigateToPose)은 받은 차례대로 쌓는다
extern std::map<std::string, std::vector<geometry_msgs::msg::PoseStamped>> g_goals;
extern std::atomic<int> g_ticks_to_finish;
// 가짜 노드의 상태 변화 "이름:RUNNING|SUCCESS|FAILURE|HALTED". BT 가 다른 스레드에서 돌 때를 위해
// 위의 map 들과 함께 g_fake_mutex 로 보호한다
extern std::vector<std::string> g_fake_events;
extern std::mutex g_fake_mutex;

int countFakeEvent(const std::string & event);
std::vector<geometry_msgs::msg::PoseStamped> fakeGoals(const std::string & node_name);
void resetFakes();

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
    const bool has_goals = static_cast<bool>(getInput("goals", goals));
    const bool has_goal = !has_goals && getInput("goal", goal);
    std::lock_guard<std::mutex> lock(g_fake_mutex);
    if (has_goals) {
      g_goals[name()] = goals;
    } else if (has_goal) {
      g_goals[name()].push_back(goal);
    }
    g_fake_events.push_back(name() + ":RUNNING");
    ticks_ = g_ticks_to_finish;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (--ticks_ > 0) {
      return BT::NodeStatus::RUNNING;
    }
    std::lock_guard<std::mutex> lock(g_fake_mutex);
    auto & queue = g_outcomes[name()];
    bool ok = true;
    if (!queue.empty()) {
      ok = queue.front();
      queue.pop_front();
    }
    g_fake_events.push_back(name() + (ok ? ":SUCCESS" : ":FAILURE"));
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    std::lock_guard<std::mutex> lock(g_fake_mutex);
    g_fake_events.push_back(name() + ":HALTED");
  }

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

// Nav2 의 NavigateThroughPoses / NavigateToPose / Spin 자리에 가짜 노드를 등록한다
inline void registerFakeNav2Nodes(BT::BehaviorTreeFactory & factory)
{
  for (const char * id : {"NavigateThroughPoses", "NavigateToPose", "Spin"}) {
    factory.registerNodeType<FakeNav2Action>(id);
  }
}

#endif  // TEST_FAKES_HPP_
