"""mission.xml 을 bt_mission 의 mission_bt_runner 로 실행한다.

  ros2 launch bt_fixed_mission mission.launch.py

mission.xml 은 이 패키지의 MakePose, MakePoses 노드를 쓰므로, 그 플러그인 라이브러리
(bt_fixed_mission_nodes)를 extra_plugin_libs 로 함께 준다. Nav2 가 뜬 뒤에 실행한다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('bt_fixed_mission')
    return LaunchDescription([
        Node(
            package='bt_mission',
            executable='mission_bt_runner',
            name='mission_bt',
            output='screen',
            parameters=[{
                'bt_xml': os.path.join(share, 'behavior_trees', 'mission.xml'),
                'extra_plugin_libs': ['bt_fixed_mission_nodes'],
            }],
        ),
    ])
