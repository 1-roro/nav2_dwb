#!/usr/bin/env python3
"""
시뮬레이션에서 로봇이 설비(dock) 앞 어디에 섰는지 Gazebo 의 참값으로 잰다. 시나리오 시험의 합격 판정에 쓴다
(bt_navigator/DOCKING.md 16장). 로봇의 odom, AMCL, 카메라를 거치지 않은 값이다.

  ros2 run simple_line_bringup dock_truth.py dock_a                  # 붙은 자리와 비교. 허용오차 안이면 종료 코드 0
  ros2 run simple_line_bringup dock_truth.py dock_b --tol 2,2,0.3    # 앞뒤 [mm], 옆 [mm], 방향 [deg]
  ros2 run simple_line_bringup dock_truth.py dock_a --target 0.71,0,180 --tol 10,5,0.5    # undock 뒤 선 자리
  ros2 run simple_line_bringup dock_truth.py dock_a --place          # 로봇을 붙은 자리에 정확히 옮겨 놓는다
                                                                     # (가르치기, aruco_teach.py 를 시험할 때)

dock 좌표계 D (설비 모형의 원점): 바닥에서 도킹 면의 가운데, x = 설비에서 로봇 쪽. 모형 이름 = dock 이름
(simple_line.launch.py 가 그렇게 놓는다). 목표는 --target 이 없으면 bt_mission 의 config/docking.yaml 에서
(docks.<dock>.gap + robot.front, 0, 180°), 그 파일이 없으면 (0.31, 0, 180°).
오차: along = 목표보다 설비에서 먼 쪽이 + (덜 들어감), lat = D 의 y (로봇이 설비를 볼 때 오른쪽이 +), yaw.
월드에 gazebo_ros_state 플러그인이 있어야 한다 (worlds/*.world 에 넣어 두었다).
"""

import argparse
import math
import os
import sys

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def target_from_config(dock):
    """bt_mission 의 docking.yaml 에서 붙은 자리 (x, y, yaw). 못 읽으면 None"""
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('bt_mission'), 'config', 'docking.yaml')
        with open(path) as f:
            cfg = yaml.safe_load(f)
        gap = cfg['docks'][dock].get('gap', 0.15)
        front = cfg.get('robot', {}).get('front', 0.16)
        return gap + front, 0.0, math.pi
    except Exception:  # noqa: BLE001  (bt_mission 이 없거나 dock 이 없으면 기본값)
        return None


def call(node, client, request, timeout=5.0):
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f'service {client.srv_name} is not available: is Gazebo running with the '
                           'gazebo_ros_state plugin in the world?')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if future.result() is None:
        raise RuntimeError(f'no answer from {client.srv_name}')
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dock', help='dock name = Gazebo model name (dock_a, dock_b)')
    parser.add_argument('--target', help='x,y,yaw_deg of the robot in the dock frame (default: docked pose)')
    parser.add_argument('--tol', default='3,3,0.5', help='along [mm], lateral [mm], yaw [deg] (default 3,3,0.5)')
    parser.add_argument('--robot', default='turtlebot3_waffle', help='robot model name in Gazebo')
    parser.add_argument('--place', action='store_true', help='move the robot to the target pose instead of measuring')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    if args.target:
        tx, ty, tyaw = (float(v) for v in args.target.split(','))
        target = (tx, ty, math.radians(tyaw))
    else:
        target = target_from_config(args.dock) or (0.31, 0.0, math.pi)
    tol = [float(v) for v in args.tol.split(',')]

    rclpy.init()
    node = rclpy.create_node('dock_truth')
    get = node.create_client(GetEntityState, 'get_entity_state')
    try:
        dock = call(node, get, GetEntityState.Request(name=args.dock, reference_frame='world'))
        if not dock.success:
            raise RuntimeError(f"no model '{args.dock}' in Gazebo "
                               '(launch with dock:=True, and dock_markers:=both for dock_b)')
        p, q = dock.state.pose.position, dock.state.pose.orientation
        dx, dy, dyaw = p.x, p.y, yaw_of(q)
        c, s = math.cos(dyaw), math.sin(dyaw)

        if args.place:
            # D 의 목표 자세 -> world
            x = dx + c * target[0] - s * target[1]
            y = dy + s * target[0] + c * target[1]
            yaw = dyaw + target[2]
            state = EntityState(name=args.robot, reference_frame='world')
            state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, 0.01
            state.pose.orientation.z, state.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            put = node.create_client(SetEntityState, 'set_entity_state')
            if not call(node, put, SetEntityState.Request(state=state)).success:
                raise RuntimeError(f"could not move '{args.robot}'")
            print(f'{args.dock}: robot placed at x {target[0] * 1000:.1f} y {target[1] * 1000:+.1f} mm '
                  f'yaw {math.degrees(target[2]):.2f} deg in the dock frame (world {x:.3f}, {y:.3f}, '
                  f'{math.degrees(yaw):.1f} deg)')
            return 0

        robot = call(node, get, GetEntityState.Request(name=args.robot, reference_frame='world'))
        if not robot.success:
            raise RuntimeError(f"no model '{args.robot}' in Gazebo")
        rp, rq = robot.state.pose.position, robot.state.pose.orientation
        ex, ey = rp.x - dx, rp.y - dy
        rx, ry = c * ex + s * ey, -s * ex + c * ey
        ryaw = angle(yaw_of(rq) - dyaw)
        along, lat = (rx - target[0]) * 1000.0, (ry - target[1]) * 1000.0
        eyaw = math.degrees(angle(ryaw - target[2]))
        ok = abs(along) <= tol[0] and abs(lat) <= tol[1] and abs(eyaw) <= tol[2]
        print(f'{args.dock}: robot x {rx * 1000:.1f} y {ry * 1000:+.1f} mm yaw {math.degrees(ryaw):.2f} deg | '
              f'error along {along:+.1f} lat {lat:+.1f} mm yaw {eyaw:+.2f} deg | '
              f"{'PASS' if ok else 'FAIL'} (tol {tol[0]:g}, {tol[1]:g} mm, {tol[2]:g} deg)")
        return 0 if ok else 1
    except RuntimeError as e:
        print(f'dock_truth: {e}')
        return 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
