#!/usr/bin/env python3
"""
4바퀴 메카넘 로봇의 URDF (TF, RViz) 와 Gazebo SDF 를 같은 치수에서 만든다. 치수를 바꾸면 이것을 다시 돌린다:

  python3 make_mecanum.py           # -> mecanum.urdf, mecanum.model (이 폴더)

로봇 (base_footprint = 네 바퀴의 가운데, 바닥 위. 메카넘은 여기를 중심으로 제자리에서 돈다):
  - 몸체 0.32 x 0.22 m, 바퀴 4개 (반지름 0.04 m, 폭 0.035 m), 바퀴까지 합친 폭 0.30 m.
    제자리에서 돌 때 쓸고 가는 반지름 0.22 m (Nav2 robot_radius 와 같다)
  - 메카넘 바퀴: 둘레에 45 도로 기운 롤러 10개. X 배치 (왼앞, 오른뒤 / 오른앞, 왼뒤가 서로 반대로 기운다)
  - 앞쪽 가운데에 RealSense (컬러 848 x 480, 69.4 도. 깊이는 같은 자리), 위 가운데에 2D 라이다, 가운데에 IMU

Gazebo 에서 (simple_line.launch.py robot:=mecanum):
  - 몸 전체가 link 하나다. gazebo_ros_planar_move 가 cmd_vel 의 vx, vy, wz 를 그대로 몸체 속도로 준다 (롤러의 마찰로
    움직이는 것을 물리로 풀지 않는다: ODE 에서 롤러 수십 개는 느리고 불안정하다). 미끄러짐은 mecanum_slip:=True 로 따로 넣는다
  - 바퀴 자리의 충돌 모양은 마찰 0 의 구(반지름 = 바퀴)라 몸을 네 점으로 받치고, base_footprint 가 바닥에 정확히 온다
  - 그래서 Gazebo 의 바퀴는 돌지 않는다. RViz 에서는 wheel_joint_publisher.py 가 odom 의 속도로 바퀴 각을 내서 돈다
"""

import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- 치수 [m] ----
BODY_L, BODY_W, BODY_H = 0.32, 0.22, 0.07       # 몸체 상자
BODY_Z = 0.035                                  # 몸체 바닥의 높이
WHEEL_R, WHEEL_W = 0.04, 0.035
WHEEL_X = 0.11                                  # 앞뒤 바퀴 축의 x (±)
WHEEL_Y = BODY_W / 2 + 0.005 + WHEEL_W / 2      # 바퀴 가운데의 y (±)
ROLLERS, ROLLER_R = 10, 0.009
DECK_Z = BODY_Z + BODY_H                        # 몸체 윗면 0.105
CAMERA = (0.15, 0.0, 0.13)                      # 컬러 카메라 (base_footprint 에서). 앞 범퍼 위 받침에
LIDAR = (0.0, 0.0, 0.16)                        # 라이다 빛의 높이
IMU = (0.0, 0.0, 0.07)

# 바퀴: 이름, x 부호, y 부호, 롤러가 기우는 쪽 (X 배치)
WHEELS = [('front_left', 1, 1, 1), ('front_right', 1, -1, -1), ('rear_left', -1, 1, -1), ('rear_right', -1, -1, 1)]


def rpy_from_matrix(R):
    pitch = math.asin(-max(-1.0, min(1.0, R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def matrix_z_to(axis):
    """z 축을 axis 로 돌리는 회전 (원기둥은 z 축 방향이다)"""
    z = axis / np.linalg.norm(axis)
    x = np.cross([0.0, 1.0, 0.0], z) if abs(z[1]) < 0.9 else np.cross([1.0, 0.0, 0.0], z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def rollers(tilt):
    """바퀴 가운데 기준 롤러들의 (x, y, z, roll, pitch, yaw). 바퀴 축 = y"""
    out = []
    for k in range(ROLLERS):
        a = 2 * math.pi * k / ROLLERS
        center = np.array([(WHEEL_R - ROLLER_R) * math.cos(a), 0.0, (WHEEL_R - ROLLER_R) * math.sin(a)])
        tangent = np.array([-math.sin(a), 0.0, math.cos(a)])
        axis = math.cos(math.pi / 4) * tangent + tilt * math.sin(math.pi / 4) * np.array([0.0, 1.0, 0.0])
        out.append((*center, *rpy_from_matrix(matrix_z_to(axis))))
    return out


def fmt(*v):
    return ' '.join(f'{x:.5f}'.rstrip('0').rstrip('.') if abs(x) > 1e-9 else '0' for x in v)


ROLLER_LEN = 2 * math.pi * (WHEEL_R - ROLLER_R) / ROLLERS * 1.25
HUB_R = WHEEL_R - 2 * ROLLER_R + 0.003

# ---------------------------------------------------------------------------------------------
# URDF

def urdf():
    half_pi = math.pi / 2
    s = ['<?xml version="1.0"?>',
         '<!-- make_mecanum.py 가 만든 4바퀴 메카넘 로봇 (치수를 바꾸면 그것을 다시 돌린다). base_footprint = 네 바퀴의 가운데, 바닥 -->',
         '<robot name="mecanum">',
         '  <material name="dark"><color rgba="0.15 0.15 0.17 1"/></material>',
         '  <material name="blue"><color rgba="0.16 0.35 0.62 1"/></material>',
         '  <material name="grey"><color rgba="0.55 0.55 0.55 1"/></material>',
         '  <material name="black"><color rgba="0.05 0.05 0.05 1"/></material>',
         '  <link name="base_footprint"/>',
         '  <joint name="base_joint" type="fixed">',
         '    <parent link="base_footprint"/><child link="base_link"/>',
         f'    <origin xyz="0 0 {fmt(WHEEL_R)}" rpy="0 0 0"/>',
         '  </joint>',
         '  <link name="base_link">',
         '    <visual>',
         f'      <origin xyz="0 0 {fmt(BODY_Z + BODY_H / 2 - WHEEL_R)}" rpy="0 0 0"/>',
         f'      <geometry><box size="{fmt(BODY_L, BODY_W, BODY_H)}"/></geometry>',
         '      <material name="blue"/>',
         '    </visual>',
         '    <collision>',
         f'      <origin xyz="0 0 {fmt(BODY_Z + BODY_H / 2 - WHEEL_R)}" rpy="0 0 0"/>',
         f'      <geometry><box size="{fmt(BODY_L, BODY_W, BODY_H)}"/></geometry>',
         '    </collision>',
         '  </link>']
    for name, sx, sy, tilt in WHEELS:
        s += [f'  <joint name="wheel_{name}_joint" type="continuous">',
              f'    <parent link="base_link"/><child link="wheel_{name}_link"/>',
              f'    <origin xyz="{fmt(sx * WHEEL_X, sy * WHEEL_Y, 0)}" rpy="0 0 0"/>',
              '    <axis xyz="0 1 0"/>',
              '  </joint>',
              f'  <link name="wheel_{name}_link">',
              '    <visual>',
              f'      <origin xyz="0 0 0" rpy="{fmt(half_pi, 0, 0)}"/>',
              f'      <geometry><cylinder radius="{fmt(HUB_R)}" length="{fmt(WHEEL_W)}"/></geometry>',
              '      <material name="grey"/>',
              '    </visual>']
        for x, y, z, r, p, yw in rollers(tilt):
            s += ['    <visual>',
                  f'      <origin xyz="{fmt(x, y, z)}" rpy="{fmt(r, p, yw)}"/>',
                  f'      <geometry><cylinder radius="{fmt(ROLLER_R)}" length="{fmt(ROLLER_LEN)}"/></geometry>',
                  '      <material name="black"/>',
                  '    </visual>']
        s += ['  </link>']
    s += ['  <joint name="imu_joint" type="fixed">',
          '    <parent link="base_link"/><child link="imu_link"/>',
          f'    <origin xyz="{fmt(IMU[0], IMU[1], IMU[2] - WHEEL_R)}" rpy="0 0 0"/>',
          '  </joint>',
          '  <link name="imu_link"/>',
          '  <joint name="scan_joint" type="fixed">',
          '    <parent link="base_link"/><child link="base_scan"/>',
          f'    <origin xyz="{fmt(LIDAR[0], LIDAR[1], LIDAR[2] - WHEEL_R)}" rpy="0 0 0"/>',
          '  </joint>',
          '  <link name="base_scan">',
          '    <visual>',
          f'      <origin xyz="0 0 {fmt(-(LIDAR[2] - DECK_Z) / 2)}" rpy="0 0 0"/>',
          f'      <geometry><cylinder radius="0.035" length="{fmt(LIDAR[2] - DECK_Z + 0.01)}"/></geometry>',
          '      <material name="dark"/>',
          '    </visual>',
          '  </link>',
          # 아래 줄 모양은 simple_line.launch.py 의 perturb_camera (camera_tf_error) 가 찾는다
          '  <joint name="camera_joint" type="fixed">',
          f'    <origin xyz="{fmt(CAMERA[0], CAMERA[1], CAMERA[2] - WHEEL_R)}" rpy="0 0 0"/>',
          '    <parent link="base_link"/><child link="camera_link"/>',
          '  </joint>',
          '  <link name="camera_link">',
          '    <visual>',
          '      <origin xyz="-0.0125 0 0" rpy="0 0 0"/>',
          '      <geometry><box size="0.025 0.09 0.025"/></geometry>',
          '      <material name="dark"/>',
          '    </visual>',
          '  </link>',
          '  <joint name="camera_rgb_joint" type="fixed">',
          '    <parent link="camera_link"/><child link="camera_rgb_frame"/>',
          '    <origin xyz="0 0 0" rpy="0 0 0"/>',
          '  </joint>',
          '  <link name="camera_rgb_frame"/>',
          '  <joint name="camera_rgb_optical_joint" type="fixed">',
          '    <parent link="camera_rgb_frame"/><child link="camera_rgb_optical_frame"/>',
          f'    <origin xyz="0 0 0" rpy="{fmt(-half_pi, 0, -half_pi)}"/>',
          '  </joint>',
          '  <link name="camera_rgb_optical_frame"/>',
          '</robot>']
    return '\n'.join(s) + '\n'


# ---------------------------------------------------------------------------------------------
# Gazebo SDF

SENSORS = '''
      <sensor name="imu" type="imu">
        <always_on>true</always_on>
        <update_rate>200</update_rate>
        <pose>{imu}</pose>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></x>
            <y><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></y>
            <z><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></x>
            <y><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></y>
            <z><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></z>
          </linear_acceleration>
        </imu>
        <plugin name="imu_driver" filename="libgazebo_ros_imu_sensor.so">
          <initial_orientation_as_reference>false</initial_orientation_as_reference>
          <frame_name>imu_link</frame_name>
          <ros><remapping>~/out:=imu</remapping></ros>
        </plugin>
      </sensor>

      <!-- 2D 라이다 (몸체 위 가운데). 빛의 높이에 제 몸이 없으므로 가까운 것도 본다 -->
      <sensor name="lidar" type="ray">
        <always_on>true</always_on>
        <visualize>false</visualize>
        <pose>{lidar}</pose>
        <update_rate>5</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1.000000</resolution>
              <min_angle>0.000000</min_angle>
              <max_angle>6.280000</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.12</min>
            <max>20.0</max>
            <resolution>0.015000</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="lidar_driver" filename="libgazebo_ros_ray_sensor.so">
          <ros><remapping>~/out:=scan</remapping></ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>base_scan</frame_name>
        </plugin>
      </sensor>

      <!-- RealSense 컬러 카메라 = URDF 의 camera_rgb_frame (base_footprint 에서 {camera_text}) -->
      <sensor name="realsense_color" type="camera">
        <always_on>true</always_on>
        <update_rate>15</update_rate>
        <pose>{camera}</pose>
        <camera name="color">
          <horizontal_fov>1.211</horizontal_fov>
          <image>
            <width>848</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.02</near>
            <far>10.0</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
        <plugin name="realsense_color_driver" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>camera</namespace>
          </ros>
          <camera_name>color</camera_name>
          <frame_name>camera_rgb_optical_frame</frame_name>
        </plugin>
      </sensor>

      <!-- 컬러와 같은 자리, 같은 화각의 깊이 = realsense-ros 의 aligned_depth_to_color (해상도만 절반) -->
      <sensor name="realsense_depth" type="depth">
        <always_on>true</always_on>
        <update_rate>15</update_rate>
        <pose>{camera}</pose>
        <camera name="depth">
          <horizontal_fov>1.211</horizontal_fov>
          <image>
            <width>424</width>
            <height>240</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>5.0</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.002</stddev>
          </noise>
        </camera>
        <plugin name="realsense_depth_driver" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>camera</namespace>
            <remapping>depth/depth/image_raw:=aligned_depth_to_color/image_raw</remapping>
            <remapping>depth/depth/camera_info:=aligned_depth_to_color/camera_info</remapping>
          </ros>
          <camera_name>depth</camera_name>
          <frame_name>camera_rgb_optical_frame</frame_name>
          <min_depth>0.1</min_depth>
          <max_depth>5.0</max_depth>
        </plugin>
      </sensor>
    </link>

    <!-- vx, vy, wz 를 그대로 몸체 속도로. odom 은 미끄러짐이 없는 참값 (mecanum_slip:=True 이면 odom_truth 로 옮긴다) -->
    <plugin name="mecanum_drive" filename="libgazebo_ros_planar_move.so">
      <ros>
        <remapping>/tf:=tf</remapping>
      </ros>
      <update_rate>50</update_rate>
      <publish_rate>30</publish_rate>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <covariance_x>0.0001</covariance_x>
      <covariance_y>0.0001</covariance_y>
      <covariance_yaw>0.01</covariance_yaw>
    </plugin>
  </model>
</sdf>
'''


def material(rgb):
    return (f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse>'
            '<specular>0.1 0.1 0.1 1</specular></material>')


def sdf():
    s = ['<?xml version="1.0"?>',
         '<!-- make_mecanum.py 가 만든 4바퀴 메카넘 로봇 (치수를 바꾸면 그것을 다시 돌린다).',
         '     몸 전체가 link 하나이고 gazebo_ros_planar_move 가 vx, vy, wz 를 그대로 몸체 속도로 준다.',
         '     바퀴 자리의 마찰 0 인 구 넷이 몸을 받친다 (base_footprint = 바닥). Gazebo 의 바퀴는 돌지 않는다 -->',
         '<sdf version="1.6">',
         '  <model name="mecanum">',
         '    <link name="base_footprint">',
         '      <inertial>',
         f'        <pose>0 0 {fmt(BODY_Z + BODY_H / 2)} 0 0 0</pose>',
         '        <mass>8.0</mass>',
         '        <inertia><ixx>0.04</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.08</iyy><iyz>0</iyz><izz>0.1</izz></inertia>',
         '      </inertial>',
         '      <collision name="body_collision">',
         f'        <pose>0 0 {fmt(BODY_Z + BODY_H / 2)} 0 0 0</pose>',
         f'        <geometry><box><size>{fmt(BODY_L, BODY_W, BODY_H)}</size></box></geometry>',
         '        <surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction></surface>',
         '      </collision>',
         '      <visual name="body_visual">',
         f'        <pose>0 0 {fmt(BODY_Z + BODY_H / 2)} 0 0 0</pose>',
         f'        <geometry><box><size>{fmt(BODY_L, BODY_W, BODY_H)}</size></box></geometry>',
         f'        {material("0.16 0.35 0.62")}',
         '      </visual>',
         '      <visual name="lidar_visual">',
         f'        <pose>{fmt(LIDAR[0], LIDAR[1], (LIDAR[2] + DECK_Z) / 2)} 0 0 0</pose>',
         f'        <geometry><cylinder><radius>0.035</radius><length>{fmt(LIDAR[2] - DECK_Z + 0.01)}</length></cylinder></geometry>',
         f'        {material("0.15 0.15 0.17")}',
         '      </visual>',
         '      <visual name="camera_visual">',
         f'        <pose>{fmt(CAMERA[0] - 0.0125, CAMERA[1], CAMERA[2])} 0 0 0</pose>',
         '        <geometry><box><size>0.025 0.09 0.025</size></box></geometry>',
         f'        {material("0.15 0.15 0.17")}',
         '      </visual>',
         '      <visual name="camera_mount_visual">',
         f'        <pose>{fmt(CAMERA[0] - 0.02, CAMERA[1], (CAMERA[2] - 0.0125 + DECK_Z) / 2)} 0 0 0</pose>',
         f'        <geometry><box><size>0.01 0.04 {fmt(CAMERA[2] - 0.0125 - DECK_Z)}</size></box></geometry>',
         f'        {material("0.15 0.15 0.17")}',
         '      </visual>']
    for name, sx, sy, tilt in WHEELS:
        x, y = sx * WHEEL_X, sy * WHEEL_Y
        s += [f'      <collision name="wheel_{name}_collision">',
              f'        <pose>{fmt(x, y, WHEEL_R)} 0 0 0</pose>',
              f'        <geometry><sphere><radius>{fmt(WHEEL_R)}</radius></sphere></geometry>',
              '        <surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction></surface>',
              '      </collision>',
              f'      <visual name="wheel_{name}_hub">',
              f'        <pose>{fmt(x, y, WHEEL_R, math.pi / 2, 0, 0)}</pose>',
              f'        <geometry><cylinder><radius>{fmt(HUB_R)}</radius><length>{fmt(WHEEL_W)}</length></cylinder></geometry>',
              f'        {material("0.55 0.55 0.55")}',
              '      </visual>']
        for k, (rx, ry, rz, rr, rp, ryw) in enumerate(rollers(tilt)):
            s += [f'      <visual name="wheel_{name}_roller_{k}">',
                  f'        <pose>{fmt(x + rx, y + ry, WHEEL_R + rz, rr, rp, ryw)}</pose>',
                  f'        <geometry><cylinder><radius>{fmt(ROLLER_R)}</radius><length>{fmt(ROLLER_LEN)}</length></cylinder></geometry>',
                  f'        {material("0.05 0.05 0.05")}',
                  '      </visual>']
    s.append(SENSORS.format(imu=f'{fmt(*IMU)} 0 0 0', lidar=f'{fmt(*LIDAR)} 0 0 0',
                            camera=f'{fmt(*CAMERA)} 0 0 0', camera_text=fmt(*CAMERA).replace(' ', ', ')).rstrip('\n'))
    return '\n'.join(s) + '\n'


def main():
    with open(os.path.join(HERE, 'mecanum.urdf'), 'w') as f:
        f.write(urdf())
    with open(os.path.join(HERE, 'mecanum.model'), 'w') as f:
        f.write(sdf())
    reach = math.hypot(BODY_L / 2, WHEEL_Y + WHEEL_W / 2)
    print(f'mecanum.urdf, mecanum.model: body {BODY_L} x {BODY_W} m, overall width {2 * (WHEEL_Y + WHEEL_W / 2):.3f} m, '
          f'turning radius {reach:.3f} m, front {BODY_L / 2:.3f} m, camera {CAMERA}')


if __name__ == '__main__':
    main()
