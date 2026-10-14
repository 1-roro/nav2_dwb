#!/usr/bin/env python3
"""
경로 추종 오차 모니터.

구독
  plan                               (nav_msgs/Path)      planner_server 가 낸 경로
  TF map -> base_link                                     로봇 위치
  navigate_to_pose/_action/status    (GoalStatusArray)    goal 시작/종료
  cmd_vel_nav                        (geometry_msgs/Twist) 컨트롤러 명령 (velocity_smoother 이전)

goal 이 끝날 때마다 출력
  - CTE vs original : goal 의 첫 경로에서 벗어난 거리 (max / RMS)
  - CTE vs latest   : 가장 최근 경로에서 벗어난 거리 (max)  <- 재계획하면 이 값만 작게 보인다
  - 최종 xy / yaw 오차, 소요 시간, 명령 선속도의 최대 증가율

파라미터
  global_frame (map), robot_frame (base_link), rate (20.0), csv_dir ('' = 저장 안 함)
"""

import csv
import math
import os

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
import tf2_ros

DONE = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def dist_to_polyline(p, pts):
    """점 p (2,) 에서 꺾은선 pts (N, 2) 까지의 최소 거리."""
    if len(pts) == 1:
        return float(np.linalg.norm(pts[0] - p))
    a, b = pts[:-1], pts[1:]
    ab = b - a
    len2 = np.maximum((ab * ab).sum(axis=1), 1e-12)
    t = np.clip(((p - a) * ab).sum(axis=1) / len2, 0.0, 1.0)
    closest = a + ab * t[:, None]
    return float(np.min(np.linalg.norm(closest - p, axis=1)))


class CteMonitor(Node):

    def __init__(self):
        super().__init__('cte_monitor')
        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').value
        rate = self.declare_parameter('rate', 20.0).value
        self.csv_dir = self.declare_parameter('csv_dir', '').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self.on_cmd, 10)
        status_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            GoalStatusArray, 'navigate_to_pose/_action/status', self.on_status, status_qos)
        self.create_timer(1.0 / rate, self.on_timer)

        self.goal_id = None
        self.finished_ids = set()
        self.reset()
        self.get_logger().info('waiting for a NavigateToPose goal ...')

    def reset(self):
        self.active = False
        self.t0 = None
        self.original = None   # 첫 경로 (N, 2)
        self.latest = None     # 최근 경로 (N, 2)
        self.goal = None       # (x, y, yaw)
        self.n_plans = 0
        self.cte_orig = []
        self.cte_latest_max = 0.0
        self.last_pose = None
        self.last_cmd = None
        self.peak_accel = 0.0
        self.rows = []

    # ---------------------------------------------------------------- callbacks

    def on_status(self, msg):
        for st in msg.status_list:
            gid = bytes(st.goal_info.goal_id.uuid)
            if st.status == GoalStatus.STATUS_EXECUTING and gid != self.goal_id \
                    and gid not in self.finished_ids:
                self.reset()
                self.goal_id = gid
                self.active = True
                self.t0 = self.get_clock().now()
                # 정지 상태에서 출발한다고 보고, 첫 명령의 0 -> v 점프도 가속으로 센다
                self.last_cmd = (self.t0, 0.0)
            elif st.status in DONE and gid == self.goal_id and gid not in self.finished_ids:
                self.finished_ids.add(gid)
                self.report(DONE[st.status])
                self.active = False

    def on_plan(self, msg):
        if not self.active or not msg.poses:
            return
        pts = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        if self.original is None:
            self.original = pts
            last = msg.poses[-1].pose
            self.goal = (last.position.x, last.position.y, yaw_of(last.orientation))
        self.latest = pts
        self.n_plans += 1

    def on_cmd(self, msg):
        if not self.active:
            return
        now = self.get_clock().now()
        if self.last_cmd is not None:
            dt = (now - self.last_cmd[0]).nanoseconds * 1e-9
            if dt > 1e-3:
                self.peak_accel = max(self.peak_accel, (msg.linear.x - self.last_cmd[1]) / dt)
        self.last_cmd = (now, msg.linear.x)

    def on_timer(self):
        if not self.active or self.original is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except tf2_ros.TransformException:
            return
        t = tf.transform.translation
        p = np.array([t.x, t.y])
        yaw = yaw_of(tf.transform.rotation)
        self.last_pose = (t.x, t.y, yaw)

        cte_o = dist_to_polyline(p, self.original)
        cte_l = dist_to_polyline(p, self.latest)
        self.cte_orig.append(cte_o)
        self.cte_latest_max = max(self.cte_latest_max, cte_l)
        if self.csv_dir:
            elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
            self.rows.append((elapsed, t.x, t.y, yaw, cte_o, cte_l))

    # ---------------------------------------------------------------- report

    def report(self, result):
        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9 if self.t0 else 0.0
        lines = [f'goal {result} in {elapsed:.1f} s ({self.n_plans} plan(s) received)']
        if self.cte_orig:
            c = np.array(self.cte_orig)
            lines.append(f'  CTE vs original path : max {c.max():.2f} m, '
                         f'rms {math.sqrt((c * c).mean()):.2f} m')
            lines.append(f'  CTE vs latest path   : max {self.cte_latest_max:.2f} m')
        if self.goal and self.last_pose:
            dx = self.last_pose[0] - self.goal[0]
            dy = self.last_pose[1] - self.goal[1]
            dyaw = math.atan2(math.sin(self.last_pose[2] - self.goal[2]),
                              math.cos(self.last_pose[2] - self.goal[2]))
            lines.append(f'  final error          : xy {math.hypot(dx, dy):.2f} m, '
                         f'yaw {math.degrees(abs(dyaw)):.1f} deg')
        lines.append(f'  peak cmd accel       : {self.peak_accel:.2f} m/s^2')
        self.get_logger().info('\n'.join(lines))

        if self.csv_dir and self.rows:
            os.makedirs(self.csv_dir, exist_ok=True)
            name = os.path.join(
                self.csv_dir, f'cte_{self.get_clock().now().nanoseconds // 10**9}.csv')
            with open(name, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'x', 'y', 'yaw', 'cte_original', 'cte_latest'])
                w.writerows(self.rows)
            self.get_logger().info(f'  trace saved to {name}')


def main():
    rclpy.init()
    node = CteMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
