#!/usr/bin/env python3
"""
여러 goal 을 한 번에 보내서, 중간 goal 들을 경유해 최종 goal 까지 가게 한다.

  ros2 run simple_line_bringup send_waypoints.py 1.0,-1.0 1.0,1.5 -1.5,1.5,180

각 인자는 map frame 의 `x,y` 또는 `x,y,yaw[deg]` 이다.
  - 중간 goal 의 yaw 를 생략하면 다음 goal 을 향하는 방향으로 채운다
  - 최종 goal 의 yaw 를 생략하면 마지막 구간의 진행 방향으로 채운다

Nav2 의 NavigateThroughPoses 액션(bt_navigator)으로 한 번에 보낸다. 그 뒤의 흐름:
  1. BT 가 3초마다 지나간 goal 을 지우고(RemovePassedGoals, 반경 0.7m)
     planner_server 에 남은 goal 들을 넘긴다 (ComputePathThroughPoses)
  2. planner_server 는 [로봇 -> goal1], [goal1 -> goal2], ... 마다 GridBased
     (= SimpleLinePlanner) 를 불러 직선들을 만들고 하나의 꺾은선 경로로 이어 붙인다
  3. controller_server 의 FollowPath (= SimpleLineController) 가 꺾은선을 따라간다.
     중간 goal 에서는 멈추지 않고, 최종 goal 에서만 멈춰서 yaw 를 맞춘다
"""

import argparse
import math
import sys

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}


def parse_waypoint(text):
    try:
        values = [float(v) for v in text.split(',')]
    except ValueError:
        values = []
    if len(values) not in (2, 3):
        raise argparse.ArgumentTypeError(f"'{text}' is not 'x,y' or 'x,y,yaw_deg'")
    return values[0], values[1], (math.radians(values[2]) if len(values) == 3 else None)


def fill_yaws(start_xy, waypoints):
    """yaw 가 비어 있는 goal 을 진행 방향으로 채운다."""
    filled = []
    points = [start_xy] + [(x, y) for x, y, _ in waypoints]
    for i, (x, y, yaw) in enumerate(waypoints):
        if yaw is None:
            if i + 1 < len(waypoints):        # 중간 goal: 다음 goal 쪽
                nx, ny = waypoints[i + 1][0], waypoints[i + 1][1]
                yaw = math.atan2(ny - y, nx - x)
            else:                             # 최종 goal: 마지막 구간 방향
                px, py = points[i]
                yaw = math.atan2(y - py, x - px)
        filled.append((x, y, yaw))
    return filled


def dist_point_segment(p, a, b):
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    len2 = vx * vx + vy * vy
    t = 0.0 if len2 == 0.0 else max(0.0, min(1.0, ((p[0] - ax) * vx + (p[1] - ay) * vy) / len2))
    return math.hypot(p[0] - (ax + t * vx), p[1] - (ay + t * vy))


def early_arrival_risk(start_xy, waypoints, tolerance):
    """
    최종 goal 근처를 경로 중간에 지나가는지 검사한다.

    Humble 의 controller_server 는 남은 경로와 상관없이 '로봇과 경로 끝점(최종 goal)의
    위치·방향 차이'만 보고 도착을 판단한다 (SimpleGoalChecker). 그래서 최종 goal 이 출발점이나
    앞 구간과 tolerance 안으로 겹치면, 그 지점에서 로봇의 방향까지 최종 goal 과 맞는 순간
    중간 goal 을 다 돌지 않았는데도 도착 처리된다. 방향이 다르면 문제없이 끝까지 가지만,
    방향은 실행 중에 정해지므로 여기서는 위치가 겹치기만 해도 막는다.
    """
    final = (waypoints[-1][0], waypoints[-1][1])
    points = [start_xy] + [(x, y) for x, y, _ in waypoints[:-1]]
    if len(points) == 1:
        return None
    for a, b in zip(points[:-1], points[1:]):
        d = dist_point_segment(final, a, b)
        if d < tolerance:
            return d
    return None


class WaypointSender(Node):

    def __init__(self):
        super().__init__('send_waypoints')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.last_remaining = None
        self.total = 0

    def robot_xy(self):
        for _ in range(100):
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', Time()).transform
                return t.translation.x, t.translation.y
            except tf2_ros.TransformException:
                continue
        raise RuntimeError('no TF map -> base_link (is Nav2 running and localized?)')

    def goal_tolerance(self, default=0.25):
        cli = self.create_client(GetParameters, '/controller_server/get_parameters')
        if not cli.wait_for_service(timeout_sec=2.0):
            return default
        fut = cli.call_async(GetParameters.Request(
            names=['general_goal_checker.xy_goal_tolerance']))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() and fut.result().values and fut.result().values[0].type == 3:
            return fut.result().values[0].double_value
        return default

    def on_feedback(self, msg):
        fb = msg.feedback
        remaining = fb.number_of_poses_remaining
        if self.last_remaining is not None and remaining < self.last_remaining:
            passed = self.total - remaining
            print(f'  passed waypoint {passed}/{self.total}  '
                  f'(distance remaining {fb.distance_remaining:.2f} m)', flush=True)
        self.last_remaining = remaining

    def send(self, waypoints):
        self.total = len(waypoints)
        goal = NavigateThroughPoses.Goal()
        stamp = self.get_clock().now().to_msg()
        for x, y, yaw in waypoints:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = stamp
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(yaw / 2)
            ps.pose.orientation.w = math.cos(yaw / 2)
            goal.poses.append(ps)

        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('navigate_through_poses action server is not up')
        handle_future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, handle_future)
        handle = handle_future.result()
        if not handle.accepted:
            print('goal rejected')
            return 1
        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            print('canceling ...')
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
            return 1
        status = STATUS.get(result_future.result().status, str(result_future.result().status))
        x, y = self.robot_xy()
        fx, fy, _ = waypoints[-1]
        print(f'result: {status}, final xy error {math.hypot(x - fx, y - fy):.2f} m')
        return 0 if status == 'SUCCEEDED' else 2


def main():
    parser = argparse.ArgumentParser(
        usage='%(prog)s [--force] x,y[,yaw_deg] [x,y[,yaw_deg] ...]',
        description="Send several goals at once (NavigateThroughPoses). Each waypoint is "
                    "map frame 'x,y' or 'x,y,yaw_deg'; the last one is the final goal.")
    parser.add_argument('--force', action='store_true',
                        help='send even if the final goal overlaps an earlier part of the route')
    # '-1.5,0.5' 처럼 '-' 로 시작하는 좌표를 옵션으로 오인하지 않도록, 옵션이 아닌 나머지를
    # 순서대로 waypoint 로 받는다
    args, rest = parser.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if not rest:
        parser.error('at least one waypoint is required')
    try:
        requested = [parse_waypoint(text) for text in rest]
    except argparse.ArgumentTypeError as e:
        parser.error(str(e))

    rclpy.init()
    node = WaypointSender()
    try:
        start = node.robot_xy()
        waypoints = fill_yaws(start, requested)

        tolerance = node.goal_tolerance()
        risk = early_arrival_risk(start, waypoints, tolerance + 0.1)
        if risk is not None:
            print(f'the final goal is only {risk:.2f} m from an earlier part of the route '
                  f'(start or a previous leg). If the robot passes there facing the final '
                  f'yaw, Nav2 Humble reports "goal reached" before the other waypoints are '
                  f'visited. Move the final goal, split the route into two commands, or use '
                  f'--force.')
            if not args.force:
                return 1

        print(f'start ({start[0]:.2f}, {start[1]:.2f})')
        for i, (x, y, yaw) in enumerate(waypoints, 1):
            kind = 'final' if i == len(waypoints) else 'via'
            print(f'  {i}. {kind:<5} ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')
        return node.send(waypoints)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
