"""
직선 경로 튜토리얼용 launch.

nav2_bringup 의 tb3_simulation_launch.py (Gazebo + TurtleBot3 waffle + Nav2 + RViz)를 그대로
쓰되, 월드/맵/스폰 위치와 params 파일만 바꿔 끼운다.
  1. 설치된 nav2_bringup/params/nav2_params.yaml (Humble 기본값)을 읽고
  2. params/line_tutorial_overrides.yaml 을 덮어쓴 뒤
  3. launch 인자(planner_step, controller_step, bt, 스폰 위치)를 반영해 임시 파일로 저장하고
  4. 그 파일을 params_file 로 넘긴다.
월드는 기본으로 worlds/line_room.world (6m x 6m 방 + 기둥 1개)를 쓴다.

예)
  ros2 launch line_tutorial_bringup tutorial.launch.py
  ros2 launch line_tutorial_bringup tutorial.launch.py planner_step:=1 controller_step:=1
  ros2 launch line_tutorial_bringup tutorial.launch.py bt:=replan
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

BT_FILES = {
    'once': 'plan_once.xml',      # 한 번만 계획 (컨트롤러 평가용)
    'replan': 'replan_1hz.xml',   # 1Hz 재계획 (기본 BT 와 같은 주기, 복구 없음)
}


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
    here = get_package_share_directory('line_tutorial_bringup')

    with open(os.path.join(nav2_bringup, 'params', 'nav2_params.yaml')) as f:
        params = yaml.safe_load(f)
    with open(os.path.join(here, 'params', 'line_tutorial_overrides.yaml')) as f:
        merge(params, yaml.safe_load(f))

    # AMCL 초기 자세 = 스폰 위치
    amcl_pose = params['amcl']['ros__parameters']['initial_pose']
    for key, arg in (('x', 'x_pose'), ('y', 'y_pose'), ('yaw', 'yaw')):
        amcl_pose[key] = float(LaunchConfiguration(arg).perform(context))

    planner_params = params['planner_server']['ros__parameters']
    controller_params = params['controller_server']['ros__parameters']
    planner = planner_params['GridBased']
    controller = controller_params['FollowPath']
    planner['step'] = int(LaunchConfiguration('planner_step').perform(context))
    controller['step'] = int(LaunchConfiguration('controller_step').perform(context))

    # 비교 실험용: BT 가 부르는 이름(GridBased / FollowPath)에 Nav2 기본 플러그인을 대신 꽂는다
    planner_choice = LaunchConfiguration('planner').perform(context)
    controller_choice = LaunchConfiguration('controller').perform(context)
    if planner_choice == 'navfn':
        planner_params['GridBased'] = dict(planner_params['NavFn'])
    elif planner_choice != 'line':
        raise RuntimeError(f"planner must be 'line' or 'navfn', got '{planner_choice}'")
    if controller_choice == 'rpp':
        controller_params['FollowPath'] = dict(controller_params['RPP'])
    elif controller_choice != 'line':
        raise RuntimeError(f"controller must be 'line' or 'rpp', got '{controller_choice}'")

    bt = LaunchConfiguration('bt').perform(context)
    if bt in BT_FILES:
        params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = \
            os.path.join(here, 'behavior_trees', BT_FILES[bt])
    elif bt != 'default':
        raise RuntimeError(f"bt must be one of {list(BT_FILES) + ['default']}, got '{bt}'")

    out = tempfile.NamedTemporaryFile(
        mode='w', prefix='line_tutorial_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, out, sort_keys=False)
    out.close()

    print(f'[line_tutorial] planner={planner_choice} (step {planner["step"]}), '
          f'controller={controller_choice} (step {controller["step"]}), bt={bt}, '
          f'params={out.name}')

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
        Node(
            condition=IfCondition(LaunchConfiguration('monitor')),
            package='line_tutorial_bringup',
            executable='cte_monitor.py',
            name='cte_monitor',
            output='screen',
            parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    here = get_package_share_directory('line_tutorial_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('planner_step', default_value='2',
                              description='0 endpoints / 1 interpolate / 2 oriented / 3 collision'),
        DeclareLaunchArgument('controller_step', default_value='4',
                              description='0 goal_seek / 1 heading_track / 2 pure_pursuit / '
                                          '3 rotate / 4 limits'),
        DeclareLaunchArgument('planner', default_value='line',
                              description="'line' (this tutorial) or 'navfn' (Nav2 NavFn)"),
        DeclareLaunchArgument('controller', default_value='line',
                              description="'line' (this tutorial) or 'rpp' (Nav2 RPP)"),
        DeclareLaunchArgument('bt', default_value='once',
                              description="'once', 'replan' or 'default' (Nav2 default BT)"),
        DeclareLaunchArgument('world',
                              default_value=os.path.join(here, 'worlds', 'line_room.world')),
        DeclareLaunchArgument('map',
                              default_value=os.path.join(here, 'maps', 'line_room.yaml')),
        DeclareLaunchArgument('x_pose', default_value='-2.0', description='spawn x (map frame)'),
        DeclareLaunchArgument('y_pose', default_value='-1.0', description='spawn y (map frame)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='spawn yaw [rad]'),
        DeclareLaunchArgument('headless', default_value='False',
                              description='True: do not start the Gazebo GUI'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument('monitor', default_value='True',
                              description='start cte_monitor.py'),
        OpaqueFunction(function=launch_setup),
    ])
