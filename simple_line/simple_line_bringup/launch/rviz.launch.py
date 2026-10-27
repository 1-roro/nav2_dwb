"""
RViz: Nav2 기본 화면에 collision_monitor 의 Footprint 원(빨강)을 더한다.

  ros2 launch simple_line_bringup rviz.launch.py                     # 실제 로봇에 붙어서 (개발 PC)
  ros2 launch simple_line_bringup rviz.launch.py use_sim_time:=True  # 시뮬레이션 (simple_line.launch.py 가 띄운다)
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

ZONES = (('Footprint', '/polygon_footprint', '255; 0; 0'),)


def launch_setup(context):
    with open(os.path.join(get_package_share_directory('nav2_bringup'),
                           'rviz', 'nav2_default_view.rviz')) as f:
        config = yaml.safe_load(f)
    for name, topic, color in ZONES:
        config['Visualization Manager']['Displays'].append({
            'Class': 'rviz_default_plugins/Polygon',
            'Name': name,
            'Enabled': True,
            'Value': True,
            'Alpha': 1,
            'Color': color,
            'Topic': {
                'Value': topic,
                'Depth': 5,
                'Durability Policy': 'Volatile',
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Reliable',
            },
        })
    out = tempfile.NamedTemporaryFile(mode='w', prefix='simple_line_', suffix='.rviz', delete=False)
    yaml.safe_dump(config, out, sort_keys=False)
    out.close()
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    return [Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', out.name],
                 parameters=[{'use_sim_time': use_sim_time}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        OpaqueFunction(function=launch_setup),
    ])
