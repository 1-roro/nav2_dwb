#!/usr/bin/env python3
"""
"가르친 자세" 만들기: 로봇을 손으로(또는 조이스틱으로) dock 에 제대로 붙여 놓고 실행한다.
그 자리에서 카메라가 본 dock 좌표계의 자세를 몇 초 동안 평균해 yaml 로 남긴다.

  ros2 run bt_mission aruco_teach.py --ros-args -p board:=dock_a
  # ~/.ros/aruco_taught/dock_a.yaml 에 남는다. 그 뒤 dock_aruco:dock_a 작업은 이것을 설 곳으로 쓴다
  # (DetectArucoDock 의 taught_dir. 지우면 다시 도면 값 docked_distance)

왜: dock 앞 몇 cm 에 서는 것을 도면 값으로 계산하면, 카메라가 로봇 어디에 어떻게 달렸는지(TF), 판의 크기와 자리,
카메라 내부값의 오차가 모두 마지막 자세 오차가 된다 (카메라 방향 1 도 = 0.33 m 앞의 판에서 6 mm).
가르친 자세를 목표로 하면 "카메라가 가르칠 때와 같은 것을 볼 때까지" 가므로, 이 오차들이 서로 지워진다
(ARUCO_DOCKING.md 5장). 로봇을 바꾸거나 카메라를 다시 달면 다시 가르친다.

남기는 것: camera_frame (detection 의 frame), position, orientation (카메라에서 본 D), 표준편차, 그때 TF 로 계산한
로봇의 자리 (dock 좌표계, 확인용). 서 있는 동안만 평균하므로 로봇을 움직이지 않는다.
"""

import math
import os
import time

import numpy as np
import yaml


def average_poses(positions, quaternions):
    """위치는 평균, 회전은 쿼터니언의 평균 (같은 반구로 맞춘 뒤 고유벡터)"""
    p = np.mean(positions, axis=0)
    q = np.array(quaternions)
    q[q @ q[0] < 0] *= -1.0
    w, v = np.linalg.eigh(q.T @ q)
    mean_q = v[:, np.argmax(w)]
    if mean_q[3] < 0:
        mean_q = -mean_q
    return p, mean_q / np.linalg.norm(mean_q)


def main():
    import rclpy
    from rclpy.node import Node
    import tf2_ros
    from bt_mission_msgs.msg import ArucoDetection

    rclpy.init()
    node = Node('aruco_teach')
    board = node.declare_parameter('board', 'dock_a').value
    output = node.declare_parameter('output', '').value or f'~/.ros/aruco_taught/{board}.yaml'
    output = os.path.expanduser(output)
    duration = node.declare_parameter('duration', 3.0).value
    robot_frame = node.declare_parameter('robot_frame', 'base_footprint').value
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node, spin_thread=True)
    samples = []
    node.create_subscription(ArucoDetection, 'dock/aruco',
                             lambda m: samples.append(m) if m.board == board else None, 10)
    node.get_logger().info(f"teaching '{board}' for {duration:.1f} s: keep the robot still at the docked pose")
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(samples) < 5:
        node.get_logger().error(f'only {len(samples)} detections of {board}: is the board in view?')
        rclpy.try_shutdown()
        os._exit(1)
    positions = np.array([[s.pose.pose.position.x, s.pose.pose.position.y, s.pose.pose.position.z] for s in samples])
    quats = [[s.pose.pose.orientation.x, s.pose.pose.orientation.y, s.pose.pose.orientation.z,
              s.pose.pose.orientation.w] for s in samples]
    p, q = average_poses(positions, quats)
    spread = positions.std(axis=0) * 1000.0
    if spread.max() > 2.0:
        node.get_logger().warn(
            f'detections spread {np.round(spread, 1)} mm: is the robot standing still at the docked pose? '
            '(at 0.33 m the spread is well under 1 mm)')
    frame = samples[0].header.frame_id
    out = {
        'board': board,
        'camera_frame': frame,
        'position': [float(v) for v in p],
        'orientation': [float(v) for v in q],
        'samples': len(samples),
        'std_position_mm': [float(v) * 1000.0 for v in positions.std(axis=0)],
        'taught_at': time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    # 확인용: 지금 TF 로 계산한 로봇의 자리 (dock 좌표계). 도면 값(예: 0.10 m 앞)과 다르면 그만큼이 TF 와 도면의 오차다
    try:
        t = tf_buffer.lookup_transform(robot_frame, frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
        tq = t.transform.rotation
        R_bc = quat_matrix([tq.x, tq.y, tq.z, tq.w])
        t_bc = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        R_bd = R_bc @ quat_matrix(q)
        t_bd = R_bc @ p + t_bc
        yaw_d = math.atan2(R_bd[1, 0], R_bd[0, 0])           # D 의 x 축을 로봇에서 본 방향
        c, s = math.cos(yaw_d), math.sin(yaw_d)
        x_r = -(c * t_bd[0] + s * t_bd[1])
        y_r = -(-s * t_bd[0] + c * t_bd[1])
        out['robot_in_dock_by_tf'] = {'x': float(x_r), 'y': float(y_r), 'yaw_deg': float(math.degrees(-yaw_d))}
        node.get_logger().info(
            f'robot in the dock frame by the current TF: x {x_r * 1000:.1f} mm, y {y_r * 1000:+.1f} mm, '
            f'yaw {math.degrees(-yaw_d):.2f} deg')
    except Exception as e:  # noqa: BLE001
        node.get_logger().warn(f'no TF {robot_frame} <- {frame}: {e}')
    os.makedirs(os.path.dirname(output) or '.', exist_ok=True)
    text = yaml.safe_dump(out, sort_keys=False)        # 먼저 글자로 (실패하면 빈 파일을 남기지 않게)
    with open(output, 'w') as f:
        f.write(text)
    node.get_logger().info(f'{len(samples)} detections, std {np.round(positions.std(axis=0) * 1000, 2)} mm -> {output}')
    rclpy.try_shutdown()
    os._exit(0)


def quat_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


if __name__ == '__main__':
    main()
