// test_mission.cpp, test_task_loop.cpp 에서 같이 쓰는 가짜 Nav2 노드와 상태 기록기

#ifndef TEST_FAKES_HPP_
#define TEST_FAKES_HPP_

#include <algorithm>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

// 가짜 Nav2 노드: g_ticks_to_finish tick 동안 RUNNING 후, 정해 둔 결과(기본 SUCCESS)를 돌려준다
inline std::map<std::string, std::deque<bool>> g_outcomes;            // 노드 이름 → 차례로 쓸 결과
inline std::map<std::string, std::vector<geometry_msgs::msg::PoseStamped>> g_goals;
inline int g_ticks_to_finish = 3;

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
    ticks_ = g_ticks_to_finish;
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

// Nav2 의 NavigateThroughPoses / NavigateToPose / Spin 자리에 가짜 노드를 등록한다
inline void registerFakeNav2Nodes(BT::BehaviorTreeFactory & factory)
{
  for (const char * id : {"NavigateThroughPoses", "NavigateToPose", "Spin"}) {
    factory.registerNodeType<FakeNav2Action>(id);
  }
}

#endif  // TEST_FAKES_HPP_
