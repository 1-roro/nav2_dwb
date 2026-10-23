"""
Gazebo + TurtleBot3 waffle + Nav2 + RViz 를 띄우고, 전역 planner 와 controller 를
simple_line_planner / simple_line_controller 로 바꿔 끼운다.

nav2_bringup 의 tb3_simulation_launch.py 를 그대로 쓴다. 설치된 Humble 기본
nav2_params.yaml 위에 params/simple_line_params.yaml 을 덮어써서 params_file 로 넘긴다.

장애물 앞에서 멈췄다 가기(stop & go)는 Nav2 의 collision_monitor 가 맡는다.

  Nav2 --cmd_vel--> collision_monitor --cmd_vel_safe--> 로봇 (Gazebo diff drive)

로봇 앞 정지 구역(params 의 StopZone)에 레이저 점이 들어오면 0 속도를 보내고, 비면 다시
Nav2 의 속도를 넘긴다. Nav2 goal 은 취소되지 않는다. 로봇이 cmd_vel_safe 를 듣도록
waffle.model 을 고친 사본으로 스폰하고, RViz 에 정지 구역(/polygon_stop)을 더한다.

  ros2 launch simple_line_bringup simple_line.launch.py
"""

import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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


def temp_file(prefix, suffix):
    return tempfile.NamedTemporaryFile(mode='w', prefix=prefix, suffix=suffix, delete=False)


def robot_sdf_listening_to(nav2_bringup, topic):
    """waffle.model 의 diff drive 플러그인이 cmd_vel 대신 topic 을 듣게 고친 사본."""
    tree = ET.parse(os.path.join(nav2_bringup, 'worlds', 'waffle.model'))
    drives = [p for p in tree.iter('plugin') if p.get('filename') == 'libgazebo_ros_diff_drive.so']
    if len(drives) != 1:
        raise RuntimeError('waffle.model 에서 diff drive 플러그인을 찾지 못했다')
    ros = drives[0].find('ros')
    if ros is None:
        ros = ET.SubElement(drives[0], 'ros')
    ET.SubElement(ros, 'remapping').text = f'cmd_vel:={topic}'
    out = temp_file('simple_line_waffle_', '.model')
    out.write(ET.tostring(tree.getroot(), encoding='unicode'))
    out.close()
    return out.name


def rviz_config_with_stop_zone(nav2_bringup):
    """Nav2 기본 RViz 설정에 collision_monitor 의 정지 구역 표시를 더한 사본."""
    with open(os.path.join(nav2_bringup, 'rviz', 'nav2_default_view.rviz')) as f:
        config = yaml.safe_load(f)
    config['Visualization Manager']['Displays'].append({
        'Class': 'rviz_default_plugins/Polygon',
        'Name': 'StopZone',
        'Enabled': True,
        'Value': True,
        'Alpha': 1,
        'Color': '255; 0; 0',
        'Topic': {
            'Value': '/polygon_stop',
            'Depth': 5,
            'Durability Policy': 'Volatile',
            'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable',
        },
    })
    out = temp_file('simple_line_', '.rviz')
    yaml.safe_dump(config, out, sort_keys=False)
    out.close()
    return out.name


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

    out = temp_file('simple_line_', '.yaml')
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
                'robot_sdf': robot_sdf_listening_to(nav2_bringup, 'cmd_vel_safe'),
                'rviz_config_file': rviz_config_with_stop_zone(nav2_bringup),
            }.items()),
        # stop & go. 설정은 params 의 collision_monitor 항목
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[out.name]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_collision_monitor',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'autostart': True,
                         'node_names': ['collision_monitor']}]),
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
