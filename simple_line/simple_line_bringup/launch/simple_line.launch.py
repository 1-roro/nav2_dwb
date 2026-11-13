"""
시뮬레이션: Gazebo + 로봇 (TurtleBot3 waffle 또는 4바퀴 메카넘) + 내비게이션(navigation.launch.py) + RViz.

  ros2 launch simple_line_bringup simple_line.launch.py                          # 빈 방, 저장된 지도로 주행
  ros2 launch simple_line_bringup simple_line.launch.py world:=warehouse slam:=True   # 창고 지도 만들기
  ros2 launch simple_line_bringup simple_line.launch.py world:=warehouse         # 창고 지도로 주행
  # ArUco 도킹 (bt_navigator/DOCKING.md): 4바퀴 메카넘 로봇 + 카메라, 창고 벽에 ArUco dock
  ros2 launch simple_line_bringup simple_line.launch.py world:=warehouse robot:=mecanum dock:=True
  # 실제 로봇과의 차이를 일부러 넣어 보기
  ros2 launch simple_line_bringup simple_line.launch.py world:=warehouse robot:=mecanum dock:=True \
      mecanum_slip:=True camera_tf_error:=1.5,0.0,0.01

실제 로봇이면 이 파일 대신 로봇 드라이버(속도 명령, odom, 레이저, robot_state_publisher)를 띄우고
navigation.launch.py 를 띄운다. 이 파일에서 시뮬레이션만의 부분은 아래 '로봇' 묶음뿐이다.

world 에 이름만 주면 worlds/<이름>.world 를 쓰고, map 을 비우면 maps/<이름>.yaml 을 쓴다.
스폰 위치를 비우면 월드마다 정해 둔 곳에 놓고, AMCL 초기 위치도 같은 값으로 준다.

robot   waffle    TurtleBot3 waffle (차동 구동, 바퀴 2개 + 캐스터. nav2_bringup 의 waffle.model)
        mecanum   4바퀴 메카넘 로봇 (robots/mecanum.urdf, mecanum.model. robots/make_mecanum.py 가 만든다):
                  몸체 0.32 x 0.30 m, 45 도 롤러 바퀴 넷, 앞쪽 RealSense, 위 라이다. Gazebo 에서는 planar_move 가
                  vx, vy, wz 를 그대로 몸체 속도로 준다. 내비게이션에 holonomic:=True 를 넘긴다 (params/holonomic.yaml).
                  holonomic 이라고 써도 같다 (예전 이름)
camera_depth:=True  컬러에 맞춘 깊이 카메라도 켠다 (기본은 끈다: ArUco 도킹은 쓰지 않고, 렌더링과 점구름 계산이 무겁다)
dock:=True        월드마다 정해 둔 곳에 ArUco marker 가 붙은 dock (models/aruco_dock, tools/make_aruco_dock.py)을
                  놓는다. 지도에는 없는 물체다
실제 로봇과의 차이를 일부러 넣기 (robot:=mecanum, bt_navigator/DOCKING.md 7장)
  mecanum_slip:=True       몸체가 명령보다 덜 가고(옆 90 %), 옆으로 가면 조금 돈다. odom 은 바퀴 odom 처럼 명령을
                           적분해서 그만큼 틀린다 (scripts/mecanum_slip_sim.py)
  camera_tf_error:=dyaw_deg,dx,dy
                           TF(robot_state_publisher)의 카메라 자리를 Gazebo 의 실제 카메라와 다르게 둔다.
                           "카메라를 도면과 조금 다르게 달았다" 를 흉내 낸다

Gazebo 가 늦게 뜨지 않게 (처음 실행이 몇 분씩 걸리던 원인)
  - Gazebo 는 model:// 로 부른 모델을 GAZEBO_MODEL_PATH 에서 못 찾으면 온라인 모델 저장소
    (GAZEBO_MODEL_DATABASE_URI, 기본 models.gazebosim.org)에서 받으려 하고, 그동안 서버 시작이 멈춘다.
    지금 그 주소는 응답이 느리거나 없어서 연결 timeout 까지 기다린다.
  - 그래서 이 launch 는 GAZEBO_MODEL_DATABASE_URI 를 비워 온라인 저장소를 쓰지 않고, 필요한 모델 경로
    (이 패키지의 models, turtlebot3_gazebo 의 models)를 GAZEBO_MODEL_PATH 에 직접 더한다.
    월드의 바닥과 해도 model:// 대신 월드 파일 안에 적었다.
"""

import math
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 월드마다 로봇을 놓는 곳 (x [m], y [m], yaw [rad])
SPAWN = {'empty_room': (-2.0, -1.0, 0.0), 'warehouse': (-4.0, -2.5, 0.0)}
# 월드마다 ArUco dock 을 놓는 곳: 충전 접점 면의 x, y, 방향(방 쪽). 동쪽 벽에 붙여 서쪽을 본다.
# marker 판이 접점 면 0.3 m 뒤에 있으므로 접점 면은 벽에서 0.45 m 떨어진다
DOCK = {'empty_room': (2.65, 1.2, math.pi), 'warehouse': (4.55, 1.0, math.pi)}


def perturb_camera(urdf, error):
    """URDF 의 camera_joint 를 "dyaw_deg,dx,dy" 만큼 옮긴다 (TF 만 바뀌고 Gazebo 의 카메라는 그대로)"""
    dyaw, dx, dy = (float(v) for v in error.split(','))
    m = re.search(r'(<joint name="camera_joint" type="fixed">\s*<origin xyz=")([^"]+)(" rpy=")([^"]+)(")', urdf)
    if not m:
        raise RuntimeError('camera_joint not found in the URDF')
    x, y, z = (float(v) for v in m.group(2).split())
    r, p, yw = (float(v) for v in m.group(4).split())
    new = f'{m.group(1)}{x + dx} {y + dy} {z}{m.group(3)}{r} {p} {yw + math.radians(dyaw)}{m.group(5)}'
    return urdf[:m.start()] + new + urdf[m.end():]


def holonomic_model(model_file, slip, depth):
    """시뮬레이션 로봇 모델을 옵션에 맞게 고친 파일 (고칠 것이 없으면 원래 파일).
    slip: planar_move 가 cmd_vel_body 를 받고, 참값 odom 을 odom_truth 로 내고, TF 는 내지 않게 (mecanum_slip_sim.py 가 낸다)
    depth: False 면 깊이 카메라를 뺀다"""
    if not slip and depth:
        return model_file
    with open(model_file) as f:
        sdf = f.read()
    if slip:
        old = '<remapping>/tf:=tf</remapping>'
        if old not in sdf:
            raise RuntimeError('unexpected holonomic model')
        sdf = sdf.replace(old, old + '\n        <remapping>cmd_vel:=cmd_vel_body</remapping>'
                          '\n        <remapping>odom:=odom_truth</remapping>', 1)
        sdf = sdf.replace('<publish_odom_tf>true</publish_odom_tf>', '<publish_odom_tf>false</publish_odom_tf>', 1)
    if not depth:
        sdf, n = re.subn(r'\s*<!--[^>]*aligned_depth_to_color[^>]*-->\s*<sensor name="realsense_depth".*?</sensor>',
                         '', sdf, flags=re.S)
        if n != 1:
            raise RuntimeError('depth camera not found in the holonomic model')
    out = tempfile.NamedTemporaryFile(mode='w', prefix='mecanum_', suffix='.model', delete=False)
    out.write(sdf)
    out.close()
    return out.name


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

    robot_type = arg('robot')
    if robot_type not in ('waffle', 'mecanum', 'holonomic'):
        raise RuntimeError(f'robot:={robot_type} 는 없다 (waffle | mecanum)')
    holonomic = robot_type in ('mecanum', 'holonomic')
    urdf = (os.path.join(here, 'robots', 'mecanum.urdf') if holonomic
            else os.path.join(nav2_bringup, 'urdf', 'turtlebot3_waffle.urdf'))
    with open(urdf) as f:
        robot_description = f.read()
    if arg('camera_tf_error'):
        robot_description = perturb_camera(robot_description, arg('camera_tf_error'))
    model = (os.path.join(here, 'robots', 'mecanum.model') if holonomic
             else os.path.join(nav2_bringup, 'worlds', 'waffle.model'))
    slip = arg('mecanum_slip').lower() == 'true'
    if slip and not holonomic:
        raise RuntimeError('mecanum_slip:=True needs robot:=mecanum')
    if holonomic:
        model = holonomic_model(model, slip, arg('camera_depth').lower() == 'true')
    model_paths = [os.path.join(here, 'models')]
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
             arguments=['-entity', 'turtlebot3_waffle', '-file', model,
                        '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', str(yaw)]),
    ]
    if arg('dock').lower() == 'true':
        if name not in DOCK:
            raise RuntimeError(f'{name} 월드에는 dock 자리를 정해 두지 않았다 (DOCK)')
        dx, dy, dyaw = DOCK[name]
        robot.append(Node(
            package='gazebo_ros', executable='spawn_entity.py', name='spawn_dock', output='screen',
            arguments=['-entity', 'aruco_dock', '-file', os.path.join(here, 'models', 'aruco_dock', 'model.sdf'),
                       '-x', str(dx), '-y', str(dy), '-z', '0', '-Y', str(dyaw)]))
    if holonomic:                           # 바퀴 넷의 각 (RViz 의 바퀴, 바퀴 TF)
        robot.append(Node(package='simple_line_bringup', executable='wheel_joint_publisher.py', output='screen',
                          parameters=[{'use_sim_time': True}]))
    if slip:
        robot.append(Node(package='simple_line_bringup', executable='mecanum_slip_sim.py', output='screen',
                          parameters=[{'use_sim_time': True}]))
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
            'holonomic': str(holonomic),
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
        DeclareLaunchArgument('robot', default_value='waffle',
                              description='waffle (차동, 바퀴 2개) | mecanum (4바퀴 메카넘 + 컬러 카메라)'),
        DeclareLaunchArgument('dock', default_value='False',
                              description='True: ArUco marker 가 붙은 dock 을 월드에 놓는다'),
        DeclareLaunchArgument('camera_depth', default_value='False',
                              description='True: 깊이 카메라도 켠다 (robot:=mecanum)'),
        DeclareLaunchArgument('mecanum_slip', default_value='False',
                              description='True: 메카넘 미끄러짐과 바퀴 odom 오차를 흉내 낸다 (robot:=mecanum)'),
        DeclareLaunchArgument('camera_tf_error', default_value='',
                              description='dyaw_deg,dx,dy: TF 의 카메라 자리를 실제(Gazebo)와 다르게'),
        OpaqueFunction(function=launch_setup),
    ])
