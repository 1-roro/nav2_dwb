// 가짜 Nav2 노드의 상태. 라이브러리 test_fake_nav2 로 빌드한다.
// 테스트가 링크해서 쓰고, BtActionServer 에는 플러그인(plugin_lib_names: test_fake_nav2)으로 준다.

#include "test_fakes.hpp"

std::map<std::string, std::deque<bool>> g_outcomes;
std::map<std::string, std::vector<geometry_msgs::msg::PoseStamped>> g_goals;
std::map<std::string, std::string> g_behavior_trees;
std::atomic<int> g_ticks_to_finish{3};
std::vector<std::string> g_fake_events;
std::mutex g_fake_mutex;

int countFakeEvent(const std::string & event)
{
  std::lock_guard<std::mutex> lock(g_fake_mutex);
  return static_cast<int>(std::count(g_fake_events.begin(), g_fake_events.end(), event));
}

std::vector<geometry_msgs::msg::PoseStamped> fakeGoals(const std::string & node_name)
{
  std::lock_guard<std::mutex> lock(g_fake_mutex);
  auto it = g_goals.find(node_name);
  return it == g_goals.end() ? std::vector<geometry_msgs::msg::PoseStamped>{} : it->second;
}

void resetFakes()
{
  std::lock_guard<std::mutex> lock(g_fake_mutex);
  g_outcomes.clear();
  g_goals.clear();
  g_behavior_trees.clear();
  g_fake_events.clear();
  g_ticks_to_finish = 3;
}

BT_REGISTER_NODES(factory)
{
  registerFakeNav2Nodes(factory);
}
