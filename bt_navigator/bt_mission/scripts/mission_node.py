#!/usr/bin/env python3
"""
behavior_trees/mission.xml 과 같은 미션을 BT 없이 Python 으로 차례로 실행한다.

  ros2 run bt_mission mission_node.py

Nav2 와는 nav2_simple_commander 의 BasicNavigator 로 주고받는다
(goThroughPoses = NavigateThroughPoses, goToPose = NavigateToPose, spin = Spin).

BT 와 대응:
  Sequence             → 위에서 아래로 차례로 실행되는 코드
  Fallback             → if 실패: 다른 처리
  RetryUntilSuccessful → for 문으로 다시 시도
  Parallel             → 주행이 끝날 때까지 도는 while 문 안에서 다른 일도 같이 처리
"""

import math
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.signals import SignalHandlerOptions

# =============================================================================================
# 미션 (map frame). (x [m], y [m], yaw [deg]), yaw 가 None 이면 진행 방향
#   점이 둘 이상이면 앞의 점들은 멈추지 않고 지나가는 경유점, 마지막 점에서 정지한다
#   (simple_line_bringup 의 빈 방 기준: 로봇은 (-2.0, -1.0) 에서 +x 를 보고 시작)
# =============================================================================================
ROUTE_A = [(0.0, -1.0, None), (1.5, -1.0, 90.0)]     # (0,-1) 경유 → 작업 지점 A
ROUTE_B = [(1.5, 1.5, None), (-1.0, 1.5, 180.0)]     # (1.5,1.5) 경유 → 작업 지점 B
HOME = [(-2.0, -1.0, 0.0)]
SCAN_INTERVAL = 0.5                                  # A → B 주행 중 스캔 간격 [m]
# =============================================================================================


def make_poses(nav, points):
    """(x, y, yaw_deg) 목록 → PoseStamped 목록. yaw 가 None 이면 진행 방향으로 채운다."""
    poses = []
    stamp = nav.get_clock().now().to_msg()
    for i, (x, y, yaw_deg) in enumerate(points):
        if yaw_deg is not None:
            yaw = math.radians(yaw_deg)
        elif i + 1 < len(points):                      # 중간 점: 다음 점 쪽
            yaw = math.atan2(points[i + 1][1] - y, points[i + 1][0] - x)
        elif i > 0:                                    # 마지막 점: 이전 점에서 온 방향
            yaw = math.atan2(y - points[i - 1][1], x - points[i - 1][0])
        else:
            raise ValueError(f'a single pose needs a yaw: {points}')
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = stamp
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        poses.append(pose)
    return poses


def wait_until_done(nav, while_running=None):
    """보낸 작업(주행, 회전)이 끝날 때까지 기다린다. 성공하면 True."""
    while not nav.isTaskComplete():                    # 0.1 초마다 돌아온다
        feedback = nav.getFeedback()
        if while_running and feedback is not None:
            while_running(feedback)
    return nav.getResult() == TaskResult.SUCCEEDED


def drive(nav, points, while_running=None):
    """점이 둘 이상이면 NavigateThroughPoses, 하나면 NavigateToPose."""
    poses = make_poses(nav, points)
    sent = nav.goThroughPoses(poses) if len(poses) > 1 else nav.goToPose(poses[0])
    return sent and wait_until_done(nav, while_running)


def spin(nav, angle):
    return nav.spin(spin_dist=angle, time_allowance=20) and wait_until_done(nav)


def do_work(task, duration):
    """정지 상태 작업 자리 (BT 의 DoWork). 실제로는 서비스 / 액션을 불러 촬영, 집기 등을 한다."""
    print(f'[work] {task}: start ({duration:.1f} s)', flush=True)
    time.sleep(duration)
    print(f'[work] {task}: done', flush=True)


class Scanner:
    """주행 중 작업 자리 (BT 의 ScanWhileMoving). interval [m] 움직일 때마다 위치를 기록한다."""

    def __init__(self, interval):
        self.interval = interval
        self.points = []

    def __call__(self, feedback):
        p = feedback.current_pose.pose.position
        if self.points and math.hypot(p.x - self.points[-1][0],
                                      p.y - self.points[-1][1]) < self.interval:
            return
        self.points.append((p.x, p.y))
        print(f'[scan] #{len(self.points)} at ({p.x:.2f}, {p.y:.2f})', flush=True)


def run_mission(nav):
    # 1. 작업 지점 A ------------------------------------------- BT: Fallback "station_a"
    if drive(nav, ROUTE_A):                                    # NavigateThroughPoses "go_a"
        do_work('inspect A', 3.0)                              # DoWork "inspect_a"
    else:
        print('[log] station A failed, skipping it')           # Log "skip_a"

    # 2. 작업 지점 B ------------------------------------------- BT: Fallback "station_b"
    scanner = Scanner(SCAN_INTERVAL)
    if (drive(nav, ROUTE_B, while_running=scanner)             # Parallel(go_b, scan)
            and spin(nav, 2 * math.pi)):                       # Spin "look_around"
        do_work('report B', 2.0)                               # DoWork "report_b"
    else:
        print('[log] station B failed, skipping it')           # Log "skip_b"
    print(f'[scan] stop: {len(scanner.points)} scans')

    # 3. 복귀 ------------------------------------------------- BT: RetryUntilSuccessful
    for _ in range(2):
        if drive(nav, HOME):                                   # NavigateToPose "go_home"
            print('[log] mission complete')                    # Log "done"
            return True
    return False


def main():
    # Ctrl-C 를 KeyboardInterrupt 로만 받는다. rclpy 기본 핸들러는 ROS 를 먼저 종료시켜서
    # 실행 중인 goal 을 취소할 수 없게 만든다.
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    nav = BasicNavigator('mission_node')
    try:
        # bt_navigator 가 active 가 될 때까지 기다린다. 기본값(localizer='amcl')은 amcl_pose 를 받기
        # 전에 initial pose 로 (0,0) 을 보내 AMCL 위치를 바꿀 수 있어서, 다른 lifecycle 노드를 준다.
        nav.waitUntilNav2Active(localizer='controller_server')
        start = time.time()
        ok = run_mission(nav)
        print(f'mission {"SUCCESS" if ok else "FAILURE"} in {time.time() - start:.1f} s')
        return 0 if ok else 1
    except KeyboardInterrupt:
        nav.cancelTask()
        return 1
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
