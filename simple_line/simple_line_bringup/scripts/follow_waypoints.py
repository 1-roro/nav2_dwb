#!/usr/bin/env python3
"""
Nav2 waypoint_follower 로 여러 goal 을 차례로 방문한다.

목표 위치는 아래 WAYPOINTS 에서 정하고, 이 파일을 실행하면 된다.

  ros2 run simple_line_bringup follow_waypoints.py
  # 또는 (WAYPOINTS 를 고친 뒤 다시 빌드하지 않고 바로 실행)
  python3 ~/nav2_ws/src/nav2_dwb/simple_line/simple_line_bringup/scripts/follow_waypoints.py

명령이 흘러가는 순서:
  이 스크립트 ──(FollowWaypoints: poses[])──▶ waypoint_follower
      waypoint_follower 가 goal 을 하나씩 꺼내서
      ──(NavigateToPose: pose)──▶ bt_navigator            (goal 마다 한 번씩, 순서대로)
          ├ ComputePathToPose ──▶ planner_server    = SimpleLinePlanner (C++)    로봇 → 이번 goal 직선
          └ FollowPath        ──▶ controller_server = SimpleLineController (C++) 그 직선을 추종
      goal 에 도착(위치 + yaw)하면 WaitAtWaypoint(기본 0.2초) 후 다음 goal 로 넘어간다
"""

import math
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

# =============================================================================================
# 목표 위치 (map frame). 위에서부터 차례로 방문한다.
#   (x [m], y [m], yaw [deg])
#   yaw 를 None 으로 두면: 중간 goal 은 다음 goal 을 향하는 방향, 마지막 goal 은 진행 방향
#   (simple_line_bringup 의 빈 방 기준: 로봇은 (-2.0, -1.0) 에서 +x 를 보고 시작, 벽은 ±3.0)
# =============================================================================================
WAYPOINTS = [
    (1.0, -1.0, None),
    (1.0, 1.5, None),
    (-1.5, 1.5, None),
    (-2.0, -1.0, 0.0),     # 출발점으로 돌아와 +x 방향으로 정렬
]
# =============================================================================================

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}


def fill_yaws(start_xy, waypoints):
    """yaw 가 None 인 goal 을 진행 방향으로 채운다 (도착 후 제자리 회전을 줄이기 위해)."""
    filled = []
    for i, (x, y, yaw_deg) in enumerate(waypoints):
        if yaw_deg is not None:
            yaw = math.radians(yaw_deg)
        elif i + 1 < len(waypoints):                   # 중간 goal: 다음 goal 쪽
            nx, ny = waypoints[i + 1][0], waypoints[i + 1][1]
            yaw = math.atan2(ny - y, nx - x)
        else:                                          # 마지막 goal: 직전 지점에서 온 방향
            px, py = (waypoints[i - 1][0], waypoints[i - 1][1]) if i > 0 else start_xy
            yaw = math.atan2(y - py, x - px)
        filled.append((x, y, yaw))
    return filled


class WaypointFollowerClient(Node):

    def __init__(self):
        super().__init__('follow_waypoints_client')
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.waypoints = []
        self.current = None
        self.t_current = None

    def robot_pose(self):
        """map 기준 로봇 위치 (x, y, yaw)."""
        for _ in range(100):
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_link', Time()).transform
            except tf2_ros.TransformException:
                continue
            q = tf.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return tf.translation.x, tf.translation.y, yaw
        raise RuntimeError('no TF map -> base_link (is Nav2 running and localized?)')

    def make_goal(self):
        goal = FollowWaypoints.Goal()
        stamp = self.get_clock().now().to_msg()
        for x, y, yaw in self.waypoints:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = stamp
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.z = math.sin(yaw / 2)
            ps.pose.orientation.w = math.cos(yaw / 2)
            goal.poses.append(ps)
        return goal

    def on_feedback(self, msg):
        # waypoint_follower 는 지금 가고 있는 goal 의 index 를 feedback 으로 계속 보낸다
        index = msg.feedback.current_waypoint
        if index == self.current:
            return
        now = time.time()
        if self.current is not None:
            print(f'  [{self.current + 1}/{len(self.waypoints)}] done in '
                  f'{now - self.t_current:.1f} s', flush=True)
        x, y, yaw = self.waypoints[index]
        print(f'  [{index + 1}/{len(self.waypoints)}] heading to '
              f'({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)', flush=True)
        self.current = index
        self.t_current = now

    def run(self):
        x0, y0, _ = self.robot_pose()
        self.waypoints = fill_yaws((x0, y0), WAYPOINTS)
        print(f'start ({x0:.2f}, {y0:.2f}), {len(self.waypoints)} waypoints')

        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('follow_waypoints action server is not up '
                               '(waypoint_follower is started by nav2_bringup)')
        t0 = time.time()
        handle_future = self.client.send_goal_async(
            self.make_goal(), feedback_callback=self.on_feedback)
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
            rclpy.spin_until_future_complete(self, handle.cancel_goal_async(), timeout_sec=5.0)
            return 1

        response = result_future.result()
        status = STATUS.get(response.status, str(response.status))
        if self.current is not None:
            print(f'  [{self.current + 1}/{len(self.waypoints)}] done in '
                  f'{time.time() - self.t_current:.1f} s')
        missed = list(response.result.missed_waypoints)
        x, y, yaw = self.robot_pose()
        fx, fy, fyaw = self.waypoints[-1]
        dyaw = math.degrees(abs(math.atan2(math.sin(yaw - fyaw), math.cos(yaw - fyaw))))
        print(f'result: {status} in {time.time() - t0:.1f} s, '
              f'missed waypoints: {[i + 1 for i in missed] if missed else "none"}')
        print(f'final error: xy {math.hypot(x - fx, y - fy):.2f} m, yaw {dyaw:.1f} deg')
        return 0 if status == 'SUCCEEDED' and not missed else 2


def main():
    rclpy.init(args=sys.argv)
    node = WaypointFollowerClient()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
