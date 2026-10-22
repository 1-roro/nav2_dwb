// ExecuteTask 액션 서버의 BtActionServer 버전 (lifecycle 노드). 하는 일은 task_server 와 같다.
//
//   ros2 launch bt_mission task_server_lifecycle.launch.py      # lifecycle_manager 가 configure → activate
//
//   ros2 run bt_mission task_server_lifecycle                   # 직접 상태를 바꿀 때
//   ros2 lifecycle set /task_server_lifecycle configure
//   ros2 lifecycle set /task_server_lifecycle activate
//
// Ctrl-C: nav2_util::LifecycleNode 가 ROS 종료 직전에 deactivate → cleanup 을 해서, 실행 중인 작업을
// 멈추고(Nav2 goal 취소) 클라이언트에 ABORTED("task server deactivated") 를 보낸다.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "bt_mission/task_server_lifecycle.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bt_mission::TaskServerLifecycle>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
