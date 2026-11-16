#!/usr/bin/env python3
"""
로봇이 명령한 만큼 실제로 움직이는지 marker 로 잰다 -> docking.yaml 의 motion 에 넣을 값 (DOCKING.md 14-3).

  # 로봇을 marker 정면 0.5 ~ 0.7 m 에 세운다 (옆, 뒤로 0.3 m 는 비어 있어야 한다). aruco_detector 가 떠 있어야 한다
  ros2 run bt_mission dock_calibrate.py                       # 배율만 (약 1 분)
  ros2 run bt_mission dock_calibrate.py --ros-args -p deadband:=True   # 데드밴드도 (2 ~ 3 분 더)

재는 것
  scale_vx, scale_vy, scale_wz   바퀴 odom 이 말하는 이동 / 실제 이동. 축마다 정해진 거리만큼 갔다가 돌아오며
                                 (앞뒤 0.10 m, 옆 0.15 m, 회전 12 도) 서서 잰 것과 비교한다. 양쪽 방향의 평균.
                                 모터 제어기가 명령한 속도를 따라가므로 (바퀴는 명령대로 돈다) 이것이 곧
                                 "명령한 속도 / 실제로 나온 속도" 다
어떻게 재나
  바퀴가 간 만큼: odom 토픽의 자세가 바뀐 양 (odom_topic). 보낸 명령을 직접 적분하면 명령이 나가는 주기와
        속도 다듬기(velocity_smoother)만큼 틀린다 (시뮬레이션에서 회전 12 % 를 잘못 셌다)
  실제 위치: 로봇에서 본 marker 의 자리 (방위와 거리. mm 단위로 정확하다)
  회전: IMU 의 yaw (imu_topic). marker 하나로 잰 dock 의 방향은 0.5 m 밖에서 1 도 넘게 틀리고, 0.7 m 지렛대로
        옆 12 mm 오차가 되어 배율을 8 % 나 틀리게 했다. IMU 가 없으면 marker 의 방향을 쓰고 경고한다 (덜 정확)
  odom_slip                      1 - (옆 효율). 필터가 바퀴 odom 을 덜 믿는 정도
  odom_yaw_per_m                 옆으로 갈 때 방향이 틀어진 양 [rad/m]
  min_vx, min_vy, min_wz         (deadband:=True) 명령을 조금씩 올려 가며 처음 움직이기 시작한 속도 + 여유

로봇이 움직인다. 주변을 비우고, 비상정지를 손에 둔다. marker 를 1 초 넘게 놓치면 멈추고 끝낸다.
결과는 화면과 ~/.ros/dock_calibration.yaml 에 남긴다. docking.yaml 의 motion 에 옮겨 적는다 (자동으로 고치지 않는다).
"""

import math
import os
import time

import yaml


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def moved(start, end):
    """서서 잰 두 번 (marker 의 자리 x, y 와 방향 yaw (base 에서), IMU yaw 또는 None) -> 처음 로봇 좌표계에서의
    이동 (앞, 왼쪽, 회전). 움직이지 않는 marker 를 두 자리에서 보았다: p0 = d + R(turn) p1"""
    if start[3] is not None and end[3] is not None:
        turn = wrap(end[3] - start[3])
    else:
        turn = wrap(-(end[2] - start[2]))               # marker 가 반대로 돈 만큼 (덜 정확)
    c, s = math.cos(turn), math.sin(turn)
    return start[0] - (c * end[0] - s * end[1]), start[1] - (s * end[0] + c * end[1]), turn


def median(values):
    v = sorted(values)
    return v[len(v) // 2]


def main():
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from bt_mission_msgs.msg import ArucoDetection

    rclpy.init()
    node = Node('dock_calibrate')
    default_config = os.path.join(get_package_share_directory('bt_mission'), 'config', 'docking.yaml')
    config_file = os.path.expanduser(node.declare_parameter('config_file', default_config).value)
    dock = node.declare_parameter('dock', '').value
    do_deadband = node.declare_parameter('deadband', False).value
    imu_topic = node.declare_parameter('imu_topic', 'imu').value
    odom_topic = node.declare_parameter('odom_topic', 'odom').value
    output = os.path.expanduser(node.declare_parameter('output', '~/.ros/dock_calibration.yaml').value)
    with open(config_file) as f:
        config = yaml.safe_load(f) or {}
    ros = config.get('ros', {})
    cmd_pub = node.create_publisher(Twist, ros.get('cmd_vel_topic', 'cmd_vel_nav'), 10)
    samples = []                                    # (받은 시각, marker (x, y, yaw) base 에서, 카메라 <-> marker)
    imu = []                                        # (받은 시각, yaw)

    def on_detection(m):
        if m.hypotheses and (not dock or m.dock == dock):
            h = m.hypotheses[0]
            samples.append((now(), (h.x, h.y, h.yaw), m.camera_distance))

    def on_imu(m):
        q = m.orientation
        imu.append((now(), math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))))
        del imu[:-400]

    if imu_topic:
        node.create_subscription(Imu, imu_topic, on_imu, qos_profile_sensor_data)
    odom = []                                       # 바퀴 odom 의 최근 자세 (x, y, yaw)

    def on_odom(m):
        q = m.pose.pose.orientation
        odom[:] = [(m.pose.pose.position.x, m.pose.pose.position.y,
                    math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))]

    node.create_subscription(Odometry, odom_topic, on_odom, 10)

    node.create_subscription(ArucoDetection, ros.get('detection_topic', 'dock/aruco'), on_detection, 10)
    log = node.get_logger()

    def now():
        # 노드의 시계 (use_sim_time 이면 시뮬레이션 시간): 낸 명령을 적분할 때 로봇과 같은 시간으로
        return node.get_clock().now().nanoseconds * 1e-9

    class Lost(Exception):
        pass

    def spin(seconds):
        end = now() + seconds
        while now() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def publish(vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
        cmd_pub.publish(t)

    def stop():
        for _ in range(5):
            publish()
            spin(0.02)

    def measure(seconds=1.0, settle=0.8):
        """서서 잰 것 (중앙값): marker 의 자리 x, y, 방향 yaw (base 에서), IMU yaw (없으면 None)"""
        stop()
        spin(settle)
        start = now()
        spin(seconds)
        recent = [s[1] for s in samples if s[0] >= start]
        if len(recent) < 3:
            raise Lost('the marker is not seen while standing')
        yaws = [y for t, y in imu if t >= start]
        yaw0 = yaws[0] if yaws else 0.0
        imu_yaw = wrap(yaw0 + median([wrap(y - yaw0) for y in yaws])) if len(yaws) >= 3 else None
        ref = recent[0][2]
        if not odom:
            raise Lost(f'no wheel odometry on {odom_topic}')
        return (median([r[0] for r in recent]), median([r[1] for r in recent]),
                wrap(ref + median([wrap(r[2] - ref) for r in recent])), imu_yaw, odom[0])

    def wheel_moved(start, end):
        """바퀴 odom 이 말하는 이동 (처음 로봇 좌표계에서 앞, 왼쪽, 회전)"""
        a, b = start[4], end[4]
        c, s = math.cos(a[2]), math.sin(a[2])
        dx, dy = b[0] - a[0], b[1] - a[1]
        return c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])

    def drive(axis, speed, distance, accel):
        """축 하나로 대략 distance 만큼 (사다리꼴 속도). 얼마나 갔는지는 바퀴 odom 과 marker 로 따로 잰다"""
        commanded, v, last = 0.0, 0.0, now()
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)
            t = now()
            dt, last = t - last, t
            if samples and t - samples[-1][0] > 1.0:
                raise Lost('lost the marker while moving')
            if samples and samples[-1][2] < 0.25:
                raise Lost('too close to the marker')
            remaining = distance - abs(commanded)
            if remaining <= 0.0:
                break
            target = min(abs(speed), math.sqrt(2.0 * accel * remaining) + 0.2 * abs(speed))
            v = min(v + accel * dt, target)
            commanded += math.copysign(v, speed) * dt
            publish(**{axis: math.copysign(v, speed)})
        stop()
        return commanded

    result = {}
    try:
        log.info('waiting for the marker ...')
        spin(3.0)
        start = measure()
        distance = samples[-1][2]
        log.info(f'marker {start[0] * 1000:.0f} mm ahead, {start[1] * 1000:+.0f} mm to the left, camera {distance:.2f} m '
                 f'from it; heading from {"the IMU (" + imu_topic + ")" if start[3] is not None else "the marker"}')
        if start[3] is None:
            log.warn('no IMU: the heading comes from the marker, which is a few times less accurate here. '
                     'Set imu_topic, or stand closer (0.4 - 0.5 m)')
        if not 0.40 <= distance <= 0.90:
            raise Lost(f'stand 0.5 - 0.7 m in front of the marker (camera is {distance:.2f} m away)')

        # ---- 배율: 갔다가 돌아오며 양쪽 방향을 잰다. 앞뒤는 뒤로 먼저 (설비에서 멀어지는 쪽)
        plan = [('vx', 'scale_vx', -0.05, 0.10, 0.1, 0), ('vy', 'scale_vy', 0.05, 0.15, 0.1, 1),
                ('wz', 'scale_wz', 0.25, math.radians(12.0), 0.5, 2)]
        for axis, name, speed, amount, accel, index in plan:
            ratios, drift = [], []
            for direction in (1.0, -1.0):
                before = measure()
                drive(axis, direction * speed, amount, accel)
                after = measure()
                d = moved(before, after)
                commanded = wheel_moved(before, after)[index]
                if abs(d[index]) < 0.2 * amount or abs(commanded) < 0.2 * amount:
                    raise Lost(f'{axis}: wheel odometry says {commanded:.4f}, the marker says {d[index]:.4f}: '
                               'is cmd_vel reaching the base, is odom_topic the wheel odometry?')
                ratios.append(commanded / d[index])
                if axis == 'vy':
                    drift.append(d[2] / d[1])           # 옆으로 1 m 갈 때 도는 각 [rad/m]
                unit = 'deg' if axis == 'wz' else 'mm'
                k = math.degrees(1.0) if axis == 'wz' else 1000.0
                log.info(f'{axis} {direction * speed:+.2f}: wheels {commanded * k:+.1f} {unit}, robot {d[index] * k:+.1f} {unit}'
                         f' (sideways {d[1 if index != 1 else 0] * 1000:+.1f} mm, turned {math.degrees(d[2]):+.2f} deg)')
            result[name] = round(sum(ratios) / len(ratios), 3)
            if axis == 'vy':
                result['odom_slip'] = round(max(0.02, abs(1.0 - 1.0 / result[name])), 3)
                result['odom_yaw_per_m'] = round(max(0.01, max(abs(v) for v in drift)), 3)

        # ---- 데드밴드: 명령을 조금씩 올리며 처음 움직인 속도
        if do_deadband:
            steps = {'vx': ('min_vx', [-0.002 * k for k in range(1, 16)], 0, 0.0005),
                     'vy': ('min_vy', [0.002 * k for k in range(1, 16)], 1, 0.0005),
                     'wz': ('min_wz', [0.005 * k for k in range(1, 16)], 2, math.radians(0.05))}
            for axis, (name, speeds, index, threshold) in steps.items():
                found = None
                for speed in speeds:
                    before = measure(0.6, 0.4)
                    end = now() + 1.0
                    while now() < end:
                        publish(**{axis: speed})
                        spin(0.02)
                    after = measure(0.6, 0.4)
                    if abs(moved(before, after)[index]) > threshold:
                        found = abs(speed)
                        break
                if found is None:
                    log.warn(f'{axis}: no motion up to {abs(speeds[-1])}: check the base')
                    continue
                result[name] = round(1.5 * found, 4)        # 여유 50 %
                log.info(f'{axis}: starts moving at {found:.4f} -> {name} {result[name]}')
    except Lost as e:
        stop()
        log.error(f'stopped: {e}')
    finally:
        stop()

    if result:
        os.makedirs(os.path.dirname(output) or '.', exist_ok=True)
        with open(output, 'w') as f:
            yaml.safe_dump({'motion': result, 'measured_at': time.strftime('%Y-%m-%d %H:%M:%S')}, f, sort_keys=False)
        print('\n# docking.yaml 의 motion 에 옮겨 적는다 (' + output + ')\nmotion:')
        for key, value in result.items():
            print(f'  {key}: {value}')
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
