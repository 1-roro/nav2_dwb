"""도킹에 쓰는 aruco_detector.py (카메라 영상 -> dock/aruco).

  시뮬레이션 (simple_line.launch.py robot:=mecanum dock:=True 가 이것을 같이 띄운다):
    ros2 launch bt_mission aruco_detector.launch.py use_sim_time:=True
  RealSense (realsense-ros 4.x 는 토픽이 /camera/camera/... 다. 카메라는 realsense.launch.py 로 띄운다):
    ros2 launch bt_mission aruco_detector.launch.py camera:=/camera/camera config_file:=$HOME/docking.yaml
    ros2 launch bt_mission aruco_detector.launch.py camera:=/camera/camera use_depth:=True   # 깊이로 방향 보정

토픽: <camera>/color/image_raw, <camera>/color/camera_info, (use_depth) <camera>/aligned_depth_to_color/image_raw.
카메라가 로봇 어디에 달렸는지는 TF (base_frame <- 영상의 frame_id) 로 안다.
dock 마다 marker 번호, 크기, 자리는 config_file (기본: bt_mission/config/docking.yaml) 의 docks. 도킹 BT 노드와 같은 파일.
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
            # 사용자 site-packages (~/.local/lib/python3.x/site-packages) 를 보지 않는다. pip install --user 로 깔린
            # numpy 2.x 가 apt 의 numpy 1.21.5 를 가리면, numpy 1.x 로 빌드된 OpenCV 4.5.4 와 cv_bridge 를 못 불러온다
            additional_env={'PYTHONNOUSERSITE': '1'},
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'base_frame': LaunchConfiguration('base_frame'),
                'marker_scale': LaunchConfiguration('marker_scale'),
                'pixel_sigma': LaunchConfiguration('pixel_sigma'),
                'stamp_offset': LaunchConfiguration('stamp_offset'),
                'depth_min_range': LaunchConfiguration('depth_min_range'),
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
        DeclareLaunchArgument('config_file', default_value=os.path.join(
            get_package_share_directory('bt_mission'), 'config', 'docking.yaml'),
            description='docking.yaml (dictionary, docks). The same file the docking BT nodes read'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
                              description='robot frame on the floor: dock poses are published in it'),
        DeclareLaunchArgument('marker_scale', default_value='1.0',
                              description='multiplies every marker size in the config (to test a size error)'),
        DeclareLaunchArgument('pixel_sigma', default_value='0.5',
                              description='corner noise used for the pose covariance [px]'),
        DeclareLaunchArgument('stamp_offset', default_value='0.0',
                              description='added to the image stamp [s] (negative if the driver stamps late)'),
        DeclareLaunchArgument('use_depth', default_value='False',
                              description='also use the aligned depth image for the dock heading'),
        DeclareLaunchArgument('depth_min_range', default_value='0.4',
                              description='depth closer than this is ignored [m] (D455: valid from about 0.4 to 0.5 m)'),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        detector([camera, '/aligned_depth_to_color/image_raw'], IfCondition(use_depth)),
        detector('', UnlessCondition(use_depth)),
    ])
