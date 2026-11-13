#!/usr/bin/env python3
"""
"가르친 자세" 만들기: 로봇을 손으로(또는 조이스틱으로) dock 에 제대로 붙여 놓고 실행한다.
그 자리에서 aruco_detector.py 가 본 dock 의 자세(base 에서 x, y, yaw)를 몇 초 동안 합쳐 yaml 로 남긴다.

  ros2 run bt_mission aruco_teach.py --ros-args -p dock:=dock_a
  # ~/.ros/aruco_taught/dock_a.yaml 에 남는다. 그 뒤 dock:dock_a 작업은 이것을 설 곳으로 쓴다
  # (DetectArucoDock 의 taught_dir. 지우면 다시 도면 값 docked_distance)

왜: dock 앞 몇 cm 에 서는 것을 도면 값으로 계산하면, 카메라가 로봇 어디에 어떻게 달렸는지(TF), marker 의 크기와
자리, 카메라 내부값, 모서리 검출의 편향(서브픽셀 모서리는 안쪽으로 0.1 ~ 0.3 px 쏠린다)이 모두 마지막 자세 오차가
된다 (카메라 방향 1 도 = 0.4 m 지렛대에서 7 mm). 가르친 자세를 목표로 하면 "detector 가 가르칠 때와 같은 것을
볼 때까지" 가므로, 이 오차들이 서로 지워진다 (DOCKING.md 6장). 로봇을 바꾸거나 카메라를 다시 달면 다시 가르친다.

합치기: 후보가 둘인 장(평면 PnP 의 뒤집힘)이 있어도, 먼저 장마다 첫 후보의 중앙값을 잡고 장마다 그것에 가까운
후보를 골라 정보 가중 평균한다 (BT 의 fuseDetections 와 같다).
"""

import math
import os
import time

import numpy as np
import yaml


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def fuse(detections):
    """detections: 장마다 [(q (3,), cov (3x3))] -> (q, cov, 쓴 장 수)"""
    first = np.array([d[0][0] for d in detections])
    ref = first[0, 2]
    mid = np.array([np.median(first[:, 0]), np.median(first[:, 1]),
                    angle(ref + np.median([angle(v - ref) for v in first[:, 2]]))])
    info, weighted, used = np.zeros((3, 3)), np.zeros(3), 0
    for hs in detections:
        def distance(h):
            e = h[0] - mid
            e[2] = angle(e[2])
            return e @ np.linalg.solve(h[1], e)
        q, cov = min(hs, key=distance)
        if distance((q, cov)) > 16.3:              # 자유도 3, 99.9 %
            continue
        e = q - mid
        e[2] = angle(e[2])
        w = np.linalg.inv(cov)
        info += w
        weighted += w @ e
        used += 1
    cov = np.linalg.inv(info)
    q = mid + cov @ weighted
    q[2] = angle(q[2])
    return q, cov, used


def main():
    import rclpy
    from rclpy.node import Node
    from bt_mission_msgs.msg import ArucoDetection

    rclpy.init()
    node = Node('aruco_teach')
    dock = node.declare_parameter('dock', 'dock_a').value
    output = node.declare_parameter('output', '').value or f'~/.ros/aruco_taught/{dock}.yaml'
    output = os.path.expanduser(output)
    duration = node.declare_parameter('duration', 3.0).value
    samples = []

    def on_detection(m):
        if m.dock == dock and m.hypotheses:
            samples.append((m.header.frame_id, [(np.array([h.x, h.y, h.yaw]), np.array(h.covariance).reshape(3, 3))
                                                for h in m.hypotheses]))

    node.create_subscription(ArucoDetection, 'dock/aruco', on_detection, 10)
    node.get_logger().info(f"teaching '{dock}' for {duration:.1f} s: keep the robot still at the docked pose")
    end = time.time() + 10.0                            # 첫 검출을 기다린다 (detector 와 이어지는 데 1 ~ 2 초)
    while not samples and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    samples.clear()
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(samples) < 5:
        node.get_logger().error(f'only {len(samples)} detections of {dock}: is the marker in view?')
        rclpy.try_shutdown()
        os._exit(1)
    q, cov, used = fuse([s[1] for s in samples])
    best = np.array([s[1][0][0] for s in samples])
    spread = np.array([best[:, 0].std() * 1000.0, best[:, 1].std() * 1000.0, math.degrees(best[:, 2].std())])
    if spread[0] > 2.0 or spread[1] > 2.0:
        node.get_logger().warn(
            f'detections spread x {spread[0]:.1f} y {spread[1]:.1f} mm: is the robot standing still at the docked pose?')
    c, s = math.cos(q[2]), math.sin(q[2])
    robot = [-(c * q[0] + s * q[1]), -(-s * q[0] + c * q[1]), angle(-q[2])]
    out = {
        'dock': dock,
        'base_frame': samples[0][0],
        'dock_in_base': {'x': float(q[0]), 'y': float(q[1]), 'yaw_deg': float(math.degrees(q[2]))},
        # 확인용: 로봇이 dock 좌표계에서 어디 있나 (도면 값, 예: 0.10 m 앞과 다르면 그만큼이 도면/TF 와 실제의 차이)
        'robot_in_dock': {'x': float(robot[0]), 'y': float(robot[1]), 'yaw_deg': float(math.degrees(robot[2]))},
        'samples': len(samples),
        'used': used,
        'sigma_mm_mm_deg': [float(math.sqrt(cov[0, 0]) * 1000.0), float(math.sqrt(cov[1, 1]) * 1000.0),
                            float(math.degrees(math.sqrt(cov[2, 2])))],
        'taught_at': time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    os.makedirs(os.path.dirname(output) or '.', exist_ok=True)
    text = yaml.safe_dump(out, sort_keys=False)        # 먼저 글자로 (실패하면 빈 파일을 남기지 않게)
    with open(output, 'w') as f:
        f.write(text)
    node.get_logger().info(
        f'{used} of {len(samples)} detections: robot in the dock frame x {robot[0] * 1000:.1f} mm, '
        f'y {robot[1] * 1000:+.1f} mm, yaw {math.degrees(robot[2]):+.2f} deg -> {output}')
    rclpy.try_shutdown()
    os._exit(0)


if __name__ == '__main__':
    main()
