"""
시뮬레이션: Gazebo + TurtleBot3 waffle + 내비게이션(navigation.launch.py) + RViz.

  ros2 launch simple_line_bringup simple_line.launch.py                          # 빈 방, 저장된 지도로 주행
  ros2 launch simple_line_bringup simple_line.launch.py world:=warehouse slam:=True   # 창고 지도 만들기
  ros2 launch simple_line_bringup simple_line.launch.py world:=warehouse         # 창고 지도로 주행

실제 로봇이면 이 파일 대신 로봇 드라이버(속도 명령, odom, 레이저, robot_state_publisher)를 띄우고
navigation.launch.py 를 띄운다. 이 파일에서 시뮬레이션만의 부분은 아래 '로봇' 묶음뿐이다.

world 에 이름만 주면 worlds/<이름>.world 를 쓰고, map 을 비우면 maps/<이름>.yaml 을 쓴다.
스폰 위치를 비우면 월드마다 정해 둔 곳에 놓고, AMCL 초기 위치도 같은 값으로 준다.

Gazebo 가 늦게 뜨지 않게 (처음 실행이 몇 분씩 걸리던 원인)
  - Gazebo 는 model:// 로 부른 모델을 GAZEBO_MODEL_PATH 에서 못 찾으면 온라인 모델 저장소
    (GAZEBO_MODEL_DATABASE_URI, 기본 models.gazebosim.org)에서 받으려 하고, 그동안 서버 시작이 멈춘다.
    지금 그 주소는 응답이 느리거나 없어서 연결 timeout 까지 기다린다.
  - 그래서 이 launch 는 GAZEBO_MODEL_DATABASE_URI 를 비워 온라인 저장소를 쓰지 않고, 필요한 모델 경로
    (turtlebot3_gazebo 의 models: waffle 몸체의 mesh)를 GAZEBO_MODEL_PATH 에 직접 더한다.
    월드의 바닥과 해도 model:// 대신 월드 파일 안에 적었다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 월드마다 로봇을 놓는 곳 (x [m], y [m], yaw [rad])
SPAWN = {'empty_room': (-2.0, -1.0, 0.0), 'warehouse': (-4.0, -2.5, 0.0)}


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    here = get_package_share_directory('simple_line_bringup')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    world = arg('world')
    name = os.path.splitext(os.path.basename(world))[0]
    if '/' not in world:
        world = os.path.join(here, 'worlds', world + '.world')
    map_yaml = arg('map') or os.path.join(here, 'maps', name + '.yaml')
    slam = arg('slam').lower() == 'true'
    if not slam and not os.path.exists(os.path.expanduser(map_yaml)):
        raise RuntimeError(f'지도 {map_yaml} 가 없다. 먼저 slam:=True 로 지도를 만들거나 map:= 을 준다')
    x, y, yaw = SPAWN.get(name, (0.0, 0.0, 0.0))
    x = float(arg('x_pose') or x)
    y = float(arg('y_pose') or y)
    yaw = float(arg('yaw') or yaw)

    with open(os.path.join(nav2_bringup, 'urdf', 'turtlebot3_waffle.urdf')) as f:
        robot_description = f.read()
    model_paths = []
    try:                                    # 로봇 몸체의 mesh (model://turtlebot3_common)
        model_paths.append(os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models'))
    except Exception:  # noqa: BLE001
        pass

    # ---- 로봇 (시뮬레이션). 실제 로봇에서는 로봇 드라이버가 이 자리에 온다
    robot = [
        # 온라인 모델 저장소를 쓰지 않는다 (없는 모델을 받으려다 시작이 멈추지 않게). 필요한 모델 경로는 직접 더한다
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        *[AppendEnvironmentVariable('GAZEBO_MODEL_PATH', path) for path in model_paths],
        ExecuteProcess(cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
                            '-s', 'libgazebo_ros_factory.so', world], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': True, 'robot_description': robot_description}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'turtlebot3_waffle',
                        '-file', os.path.join(nav2_bringup, 'worlds', 'waffle.model'),
                        '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', str(yaw)]),
    ]
    if arg('headless').lower() != 'true':
        robot.append(ExecuteProcess(cmd=['gzclient'], output='screen'))

    # ---- 내비게이션 (실제 로봇과 같은 파일)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'slam': str(slam),
            'map': '' if slam else map_yaml,
            'use_sim_time': 'True',
            'initial_pose_x': str(x),
            'initial_pose_y': str(y),
            'initial_pose_yaw': str(yaw),
        }.items())
    actions = robot + [navigation]
    if arg('use_rviz').lower() == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(here, 'launch', 'rviz.launch.py')),
            launch_arguments={'use_sim_time': 'True'}.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty_room',
                              description='empty_room | warehouse | .world 파일 경로'),
        DeclareLaunchArgument('map', default_value='',
                              description='지도 yaml. 비우면 maps/<world 이름>.yaml'),
        DeclareLaunchArgument('slam', default_value='False',
                              description='True: 지도를 만든다 (slam_toolbox)'),
        DeclareLaunchArgument('x_pose', default_value='', description='스폰 x [m] (map frame)'),
        DeclareLaunchArgument('y_pose', default_value='', description='스폰 y [m] (map frame)'),
        DeclareLaunchArgument('yaw', default_value='', description='스폰 yaw [rad]'),
        DeclareLaunchArgument('headless', default_value='False',
                              description='True: Gazebo 창을 띄우지 않는다'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        OpaqueFunction(function=launch_setup),
    ])
