"""도킹에 쓰는 aruco_detector.py (카메라 영상 -> dock/aruco).

  시뮬레이션 (simple_line_bringup robot:=mecanum dock:=True):
    ros2 launch bt_mission aruco_detector.launch.py use_sim_time:=True
  RealSense (realsense-ros 4.x 는 /camera/camera/...):
    ros2 launch bt_mission aruco_detector.launch.py camera:=/camera/camera docks_file:=$HOME/my_docks.yaml

토픽: <camera>/color/image_raw, <camera>/color/camera_info. 카메라가 로봇 어디에 달렸는지는 TF (base_frame <- 영상의
frame_id) 로 안다. dock 마다 marker 번호, 크기, 자리는 docks_file (기본: bt_mission/config/aruco_docks.yaml).
인쇄한 marker 를 자로 재서 맞춘다. marker_scale: 크기에 곱한다 (시뮬레이션에서 크기 오차를 일부러 넣을 때도).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera = LaunchConfiguration('camera')
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='/camera',
                              description='camera namespace (realsense-ros 4.x: /camera/camera)'),
        DeclareLaunchArgument('docks_file', default_value=os.path.join(
            get_package_share_directory('bt_mission'), 'config', 'aruco_docks.yaml')),
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
                              description='robot frame on the floor: dock poses are published in it'),
        DeclareLaunchArgument('marker_scale', default_value='1.0',
                              description='multiplies every marker size in docks_file'),
        DeclareLaunchArgument('pixel_sigma', default_value='0.5',
                              description='corner noise used for the pose covariance [px]'),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        Node(
            package='bt_mission',
            executable='aruco_detector.py',
            name='aruco_detector',
            output='screen',
            # 사용자 site-packages (~/.local/lib/python3.x/site-packages) 를 보지 않는다. pip install --user 로 깔린
            # numpy 2.x 가 apt 의 numpy 1.21.5 를 가리면, numpy 1.x 로 빌드된 OpenCV 4.5.4 와 cv_bridge 를 못 불러온다
            additional_env={'PYTHONNOUSERSITE': '1'},
            parameters=[{
                'docks_file': LaunchConfiguration('docks_file'),
                'base_frame': LaunchConfiguration('base_frame'),
                'marker_scale': LaunchConfiguration('marker_scale'),
                'pixel_sigma': LaunchConfiguration('pixel_sigma'),
                'image_topic': [camera, '/color/image_raw'],
                'camera_info_topic': [camera, '/color/camera_info'],
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
