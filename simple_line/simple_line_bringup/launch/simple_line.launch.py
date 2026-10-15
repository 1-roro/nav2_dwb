"""
Gazebo + TurtleBot3 waffle + Nav2 + RViz 를 띄우고, 전역 planner 와 controller 를
simple_line_planner / simple_line_controller 로 바꿔 끼운다.

nav2_bringup 의 tb3_simulation_launch.py 를 그대로 쓴다. 설치된 Humble 기본
nav2_params.yaml 위에 params/simple_line_params.yaml 을 덮어써서 params_file 로 넘긴다.

  ros2 launch simple_line_bringup simple_line.launch.py
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def merge(base, over):
    """over 를 base 에 덮어쓴다. 'plugin' 키가 있는 dict 는 통째로 교체한다."""
    for key, value in over.items():
        if isinstance(value, dict) and 'plugin' in value:
            base[key] = value
        elif isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value
    return base


def launch_setup(context):
    nav2_bringup = get_package_share_directory('nav2_bringup')
    here = get_package_share_directory('simple_line_bringup')

    with open(os.path.join(nav2_bringup, 'params', 'nav2_params.yaml')) as f:
        params = yaml.safe_load(f)
    with open(os.path.join(here, 'params', 'simple_line_params.yaml')) as f:
        merge(params, yaml.safe_load(f))

    # AMCL 초기 자세 = 스폰 위치
    initial_pose = params['amcl']['ros__parameters']['initial_pose']
    for key, arg in (('x', 'x_pose'), ('y', 'y_pose'), ('yaw', 'yaw')):
        initial_pose[key] = float(LaunchConfiguration(arg).perform(context))

    out = tempfile.NamedTemporaryFile(
        mode='w', prefix='simple_line_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, out, sort_keys=False)
    out.close()

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'tb3_simulation_launch.py')),
            launch_arguments={
                'params_file': out.name,
                'world': LaunchConfiguration('world'),
                'map': LaunchConfiguration('map'),
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
                'yaw': LaunchConfiguration('yaw'),
                'headless': LaunchConfiguration('headless'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items()),
    ]


def generate_launch_description():
    here = get_package_share_directory('simple_line_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=os.path.join(here, 'worlds', 'empty_room.world')),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(here, 'maps', 'empty_room.yaml')),
        DeclareLaunchArgument('x_pose', default_value='-2.0', description='spawn x (map frame)'),
        DeclareLaunchArgument('y_pose', default_value='-1.0', description='spawn y (map frame)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='spawn yaw [rad]'),
        DeclareLaunchArgument('headless', default_value='False',
                              description='True: do not start the Gazebo GUI'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        OpaqueFunction(function=launch_setup),
    ])
