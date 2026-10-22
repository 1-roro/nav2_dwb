"""task_server_lifecycle (BtActionServer 버전) 과, 이 노드만 맡는 lifecycle_manager.

  ros2 launch bt_mission task_server_lifecycle.launch.py

lifecycle_manager 가 configure → activate 를 한다. activate 에서 트리를 만들 때 Nav2 액션 서버
(navigate_through_poses, spin 등)를 찾으므로, Nav2 가 뜬 뒤에 실행한다.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bt_mission',
            executable='task_server_lifecycle',
            name='task_server_lifecycle',
            output='screen',
            # BtActionServer 가 BT 상태 변화를 내는 토픽. bt_navigator 의 것과 섞이지 않게 옮긴다
            remappings=[('behavior_tree_log', 'mission/behavior_tree_log')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mission',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['task_server_lifecycle']}],
        ),
    ])
