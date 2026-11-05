"""QR 도킹에 쓰는 qr_detector.py (카메라 영상 -> dock/qr).

  시뮬레이션 (simple_line_bringup robot:=holonomic):
    ros2 launch bt_mission qr_detector.launch.py use_sim_time:=True
  RealSense (realsense-ros 4.x 는 /camera/camera/..., 깊이를 컬러에 맞추게 align_depth.enable:=true 로 띄운다):
    ros2 launch bt_mission qr_detector.launch.py camera:=/camera/camera qr_size:=0.1448

토픽: <camera>/color/image_raw, <camera>/color/camera_info, <camera>/aligned_depth_to_color/image_raw.
use_depth:=False 이면 깊이를 쓰지 않고 QR 네 모서리로만 방향을 구한다 (멀리서는 덜 정확하다).
qr_size 는 인쇄한 QR 의 한 변(흰 여백 제외)을 자로 잰 값이다 (tools/make_qr.py 가 계산값을 알려 준다).
"""

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
            executable='qr_detector.py',
            name='qr_detector',
            output='screen',
            condition=condition,
            parameters=[{
                'qr_size': LaunchConfiguration('qr_size'),          # 문자열이 아니라 yaml 로 읽혀 float 가 된다
                'image_topic': [camera, '/color/image_raw'],
                'camera_info_topic': [camera, '/color/camera_info'],
                'depth_topic': depth_topic,
                'optical_frame': LaunchConfiguration('optical_frame'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        )

    use_depth = LaunchConfiguration('use_depth')
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='/camera',
                              description='camera namespace (realsense-ros 4.x: /camera/camera)'),
        DeclareLaunchArgument('use_depth', default_value='True',
                              description='QR normal from the aligned depth image'),
        DeclareLaunchArgument('qr_size', default_value='0.1448',
                              description='QR side without the white margin [m]'),
        DeclareLaunchArgument('optical_frame', default_value='',
                              description='frame of the detections if the image frame_id is not the optical frame'),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        detector([camera, '/aligned_depth_to_color/image_raw'], IfCondition(use_depth)),
        detector('', UnlessCondition(use_depth)),
    ])
