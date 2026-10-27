"""
로봇에 상관없는 내비게이션 묶음. 시뮬레이션(simple_line.launch.py)과 실제 로봇이 같이 쓴다.

  # 지도 만들기 (slam_toolbox). 다 만들면: ros2 run nav2_map_server map_saver_cli -f ~/maps/<이름>
  ros2 launch simple_line_bringup navigation.launch.py slam:=True
  # 저장한 지도에서 주행 (map_server + AMCL)
  ros2 launch simple_line_bringup navigation.launch.py map:=$HOME/maps/<이름>.yaml

띄우는 것
  slam:=True    slam_toolbox                 지도를 만들면서 위치 추정 (map -> odom TF)
  slam:=False   map_server + amcl            저장한 지도에서 위치 추정 (map -> odom TF)
  공통          planner_server, controller_server, behavior_server, bt_navigator,
                waypoint_follower, collision_monitor, velocity_smoother

속도가 로봇까지 가는 길
  controller_server, behavior_server --cmd_vel_nav--> collision_monitor --cmd_vel_safe-->
  velocity_smoother --cmd_vel--> 로봇
  장애물 앞 감속은 controller 가 경로 위 장애물까지의 거리로 S 커브를 그려 하고, collision_monitor 는
  갑자기 나타난 것이 StopZone 에 들어올 때만 0 으로 바꾸는 비상 정지다. velocity_smoother 는
  로봇이 낼 수 있는 가감속 한계를 지킨다.

로봇이 줘야 하는 것: /scan, /odom, TF odom -> base_footprint -> base_scan, /cmd_vel 을 받아 움직이기.

파라미터: 설치된 Humble 기본 nav2_params.yaml 과 slam_toolbox 의 mapper_params_online_async.yaml 위에
params_file(기본 params/simple_line_params.yaml)을 덮어쓴다. 'plugin' 키가 있는 항목은 통째로 바뀐다.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

LOCALIZATION_NODES = ['map_server', 'amcl']
NAVIGATION_NODES = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator',
                    'waypoint_follower', 'collision_monitor', 'velocity_smoother']


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


def set_use_sim_time(params, value):
    """모든 노드(costmap 같은 하위 노드 포함)의 use_sim_time 을 맞춘다."""
    for key, sub in params.items():
        if key == 'ros__parameters':
            sub['use_sim_time'] = value
        elif isinstance(sub, dict):
            set_use_sim_time(sub, value)


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    here = get_package_share_directory('simple_line_bringup')
    slam = arg('slam').lower() == 'true'
    use_sim_time = arg('use_sim_time').lower() == 'true'
    map_yaml = os.path.expanduser(arg('map'))
    if not slam and not map_yaml:
        raise RuntimeError('map:=<지도 yaml> 을 주거나, 지도를 만들려면 slam:=True')

    with open(os.path.join(get_package_share_directory('nav2_bringup'),
                           'params', 'nav2_params.yaml')) as f:
        params = yaml.safe_load(f)
    with open(os.path.join(get_package_share_directory('slam_toolbox'),
                           'config', 'mapper_params_online_async.yaml')) as f:
        params.update(yaml.safe_load(f))
    with open(os.path.expanduser(arg('params_file'))) as f:
        merge(params, yaml.safe_load(f))

    set_use_sim_time(params, use_sim_time)
    # 이 패키지의 navigation BT (제자리 회전, 후진 recovery 없음)
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = os.path.join(
        here, 'behavior_trees', 'navigate_to_pose_amr.xml')
    if map_yaml:
        params['map_server']['ros__parameters']['yaml_filename'] = map_yaml
    if arg('initial_pose_x') and arg('initial_pose_y'):
        amcl = params['amcl']['ros__parameters']
        amcl['set_initial_pose'] = True
        amcl['initial_pose'] = {
            'x': float(arg('initial_pose_x')), 'y': float(arg('initial_pose_y')), 'z': 0.0,
            'yaw': float(arg('initial_pose_yaw') or 0.0)}

    out = tempfile.NamedTemporaryFile(
        mode='w', prefix='simple_line_nav_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, out, sort_keys=False)
    out.close()

    def node(package, executable, name, remappings=()):
        return Node(package=package, executable=executable, name=name, output='screen',
                    parameters=[out.name], remappings=list(remappings))

    def lifecycle_manager(name, nodes):
        return Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name=name,
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                                 'node_names': nodes}])

    if slam:
        localization = [node('slam_toolbox', 'async_slam_toolbox_node', 'slam_toolbox')]
    else:
        localization = [
            node('nav2_map_server', 'map_server', 'map_server'),
            node('nav2_amcl', 'amcl', 'amcl'),
            lifecycle_manager('lifecycle_manager_localization', LOCALIZATION_NODES),
        ]

    navigation = [
        node('nav2_planner', 'planner_server', 'planner_server'),
        node('nav2_controller', 'controller_server', 'controller_server',
             [('cmd_vel', 'cmd_vel_nav')]),
        node('nav2_behaviors', 'behavior_server', 'behavior_server',
             [('cmd_vel', 'cmd_vel_nav')]),        # 회전, 대기 같은 동작도 같은 길로
        node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        node('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
             [('cmd_vel', 'cmd_vel_safe'), ('cmd_vel_smoothed', 'cmd_vel')]),
        lifecycle_manager('lifecycle_manager_navigation', NAVIGATION_NODES),
    ]
    return localization + navigation


def generate_launch_description():
    here = get_package_share_directory('simple_line_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='False',
                              description='True: 지도를 만든다 (slam_toolbox), False: map 으로 AMCL'),
        DeclareLaunchArgument('map', default_value='', description='지도 yaml (slam:=False 일 때)'),
        DeclareLaunchArgument('use_sim_time', default_value='False',
                              description='True: 시뮬레이션 시계 (/clock)'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(here, 'params', 'simple_line_params.yaml'),
            description='Humble 기본 nav2_params.yaml 위에 덮어쓸 파일'),
        DeclareLaunchArgument('initial_pose_x', default_value='',
                              description='AMCL 초기 위치 x [m] (비우면 RViz 의 2D Pose Estimate 로)'),
        DeclareLaunchArgument('initial_pose_y', default_value='', description='AMCL 초기 위치 y [m]'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='',
                              description='AMCL 초기 방향 [rad]'),
        OpaqueFunction(function=launch_setup),
    ])

