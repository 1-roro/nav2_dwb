"""ArUco 도킹에 쓰는 aruco_detector.py (카메라 영상 -> dock/aruco).

  시뮬레이션 (simple_line_bringup robot:=holonomic dock:=aruco):
    ros2 launch bt_mission aruco_detector.launch.py use_sim_time:=True
  RealSense (realsense-ros 4.x 는 /camera/camera/...):
    ros2 launch bt_mission aruco_detector.launch.py camera:=/camera/camera boards_file:=$HOME/my_boards.yaml

토픽: <camera>/color/image_raw, <camera>/color/camera_info, (use_depth:=True 이면) <camera>/aligned_depth_to_color/image_raw.
board 의 marker 크기와 자리는 boards_file (기본: bt_mission/config/aruco_boards.yaml). 인쇄한 판을 자로 재서 맞춘다.
board_scale: 그 값 전체에 곱한다 (시뮬레이션에서 크기 오차를 일부러 넣을 때도).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera = LaunchConfiguration('camera')

    def detector(depth_topic, condition):
        return Node(
            package='bt_mission',
            executable='aruco_detector.py',
            name='aruco_detector',
            output='screen',
            condition=condition,
            parameters=[{
                'boards_file': LaunchConfiguration('boards_file'),
                'board_scale': LaunchConfiguration('board_scale'),
                'pixel_sigma': LaunchConfiguration('pixel_sigma'),
                'image_topic': [camera, '/color/image_raw'],
                'camera_info_topic': [camera, '/color/camera_info'],
                'depth_topic': depth_topic,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        )

    use_depth = LaunchConfiguration('use_depth')
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='/camera',
                              description='camera namespace (realsense-ros 4.x: /camera/camera)'),
        DeclareLaunchArgument('boards_file', default_value=os.path.join(
            get_package_share_directory('bt_mission'), 'config', 'aruco_boards.yaml')),
        DeclareLaunchArgument('board_scale', default_value='1.0',
                              description='multiplies every marker size and position in boards_file'),
        DeclareLaunchArgument('pixel_sigma', default_value='0.5',
                              description='corner noise used for the pose covariance [px]'),
        DeclareLaunchArgument('use_depth', default_value='False',
                              description='board tilt from the aligned depth image (the board PnP is usually enough)'),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        detector([camera, '/aligned_depth_to_color/image_raw'], IfCondition(use_depth)),
        detector('', UnlessCondition(use_depth)),
    ])
