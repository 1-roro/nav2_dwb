#!/usr/bin/env python3
"""
task_loop.xml 로 실행 중인 미션 BT 에 작업을 보낸다.

  보내는 토픽: mission/task   (bt_mission_msgs/Task)
  받는 토픽  : mission/status (std_msgs/String, "<작업 id> <상태>")

  # 대화형: 한 줄에 작업 하나. 보낸 작업은 BT 가 받은 순서대로 하나씩 실행한다
  ros2 run bt_mission task_sender.py
  task> inspect 0,-1 1.5,-1,90      (0,-1) 경유 → (1.5,-1) 에서 정지 → 3초 점검
  task> scan 1.5,1.5 -1,1.5,180     가는 동안 0.5m 마다 스캔
  task> look_around -1,1.5,180      도착 후 제자리 한 바퀴
  task> go -2,-1,0                  이동만
  task> inspect:5 1,0,0             작업 시간 5초

  # 작업 하나만 보내고, 끝날 때까지 기다렸다가 종료
  ros2 run bt_mission task_sender.py inspect 0,-1 1.5,-1,90

한 줄 형식: <action>[:duration] x,y[,yaw_deg] [x,y[,yaw_deg] ...]
  - 좌표는 map frame. 마지막 점이 작업 지점(정지), 앞의 점은 경유점(통과)
  - yaw 를 생략하면 중간 점은 다음 점 쪽, 마지막 점은 직전 점에서 온 방향으로 채운다.
    작업의 첫 점은 직전 작업의 마지막 점(첫 작업이면 로봇 현재 위치)에서 온 것으로 본다
"""

import math
import select
import sys
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from bt_mission_msgs.msg import Task
from std_msgs.msg import String
import tf2_ros

ACTIONS = (Task.ACTION_GO, Task.ACTION_INSPECT, Task.ACTION_SCAN, Task.ACTION_LOOK_AROUND)
DEFAULT_DURATION = 3.0
FINAL_STATES = ('done', 'failed', 'rejected')

HELP = '''한 줄에 작업 하나: <action>[:duration] x,y[,yaw_deg] [x,y[,yaw_deg] ...]   (map frame)
  action: go | inspect | scan | look_around. 마지막 점에서 정지, 앞의 점은 경유점
  예) inspect 0,-1 1.5,-1,90    scan 1.5,1.5 -1,1.5,180    look_around -1,1.5,180    go -2,-1,0
  help: 이 도움말, quit 또는 Ctrl-D: 끝내기 (보낸 작업이 남아 있으면 끝날 때까지 기다림)'''


def parse_line(text):
    """'inspect:5 0,-1 1.5,-1,90' -> ('inspect', 5.0, [(0, -1, None), (1.5, -1, 90)])"""
    tokens = text.split()
    if len(tokens) < 2:
        raise ValueError('need an action and at least one point')
    action, _, duration = tokens[0].partition(':')
    if action not in ACTIONS:
        raise ValueError(f"unknown action '{action}' (one of {', '.join(ACTIONS)})")
    duration = float(duration) if duration else DEFAULT_DURATION
    points = []
    for token in tokens[1:]:
        try:
            values = [float(v) for v in token.split(',')]
        except ValueError:
            values = []
        if len(values) not in (2, 3):
            raise ValueError(f"'{token}' is not x,y or x,y,yaw_deg")
        points.append((values[0], values[1], values[2] if len(values) == 3 else None))
    return action, duration, points


def fill_yaws(start_xy, points):
    """yaw 가 None 인 점을 진행 방향으로 채운다 [rad]."""
    xy = [start_xy] + [(x, y) for x, y, _ in points]
    filled = []
    for i, (x, y, yaw_deg) in enumerate(points):
        if yaw_deg is not None:
            yaw = math.radians(yaw_deg)
        elif i + 1 < len(points):                       # 중간 점: 다음 점 쪽
            yaw = math.atan2(points[i + 1][1] - y, points[i + 1][0] - x)
        else:                                           # 마지막 점: 직전 점에서 온 방향
            px, py = xy[i]
            yaw = math.atan2(y - py, x - px) if (x, y) != (px, py) else 0.0
        filled.append((x, y, yaw))
    return filled


class TaskSender(Node):

    def __init__(self):
        super().__init__('task_sender')
        self.task_pub = self.create_publisher(Task, 'mission/task', 10)
        self.create_subscription(String, 'mission/status', self.on_status, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.last_xy = None          # 직전 작업의 마지막 점
        self.pending = set()         # 보냈지만 아직 끝나지 않은 작업 id
        self.watch = None            # 이 id 의 상태만 출력 (None 이면 모두)

    def on_status(self, msg):
        task_id, _, state = msg.data.partition(' ')
        if self.watch is None or task_id == self.watch:
            print(f'  <- {msg.data}', flush=True)
        if state.startswith(FINAL_STATES):
            self.pending.discard(task_id)

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time()).transform
            return t.translation.x, t.translation.y
        except tf2_ros.TransformException:
            return None

    def wait_for_bt(self, timeout=10.0):
        """WaitForTask 의 구독이 연결될 때까지 기다린다 (연결 전에 보낸 메시지는 사라진다)."""
        deadline = time.time() + timeout
        while self.task_pub.get_subscription_count() == 0:
            if time.time() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def send(self, task_id, action, duration, points):
        start = self.last_xy or self.robot_xy() or (points[0][0], points[0][1])
        filled = fill_yaws(start, points)
        task = Task(id=task_id, action=action, duration=duration)
        stamp = self.get_clock().now().to_msg()
        for x, y, yaw in filled:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            task.waypoints.append(pose)
        self.pending.add(task_id)
        self.task_pub.publish(task)
        self.last_xy = (points[-1][0], points[-1][1])
        route = ' -> '.join(f'({x:g},{y:g},{math.degrees(yaw):.0f}°)' for x, y, yaw in filled)
        print(f'  -> {task_id}: {action} {route}' +
              (f', {duration:g} s' if action == Task.ACTION_INSPECT else ''), flush=True)

    def wait_until_finished(self, task_ids=None):
        while self.pending if task_ids is None else self.pending & set(task_ids):
            rclpy.spin_once(self, timeout_sec=0.1)

    def read_line(self, prompt):
        """stdin 에서 한 줄을 읽는다. 기다리는 동안에도 상태 토픽과 TF 를 받는다."""
        print(prompt, end='', flush=True)
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                line = sys.stdin.readline()
                if not line:
                    raise EOFError
                return line
            rclpy.spin_once(self, timeout_sec=0.0)


def interactive(node):
    print(HELP)
    count = 0
    while True:
        try:
            line = node.read_line('task> ').split('#')[0].strip()
        except EOFError:                     # Ctrl-D, 또는 파이프 입력이 끝남
            break
        if not line:
            continue
        if line in ('quit', 'exit'):
            break
        if line == 'help':
            print(HELP)
            continue
        try:
            action, duration, points = parse_line(line)
        except ValueError as e:
            print(f'  ! {e}')
            continue
        if not node.wait_for_bt():
            print('  ! no subscriber on mission/task. '
                  'Is mission_bt_runner running with bt_xml:=task_loop.xml ?')
            continue
        count += 1
        node.send(f't{count}', action, duration, points)
    if node.pending:
        print(f'waiting for {len(node.pending)} task(s) to finish (Ctrl-C to quit)')
        node.wait_until_finished()


def once(node, args):
    action, duration, points = parse_line(' '.join(args))
    if not node.wait_for_bt():
        print('no subscriber on mission/task. '
              'Is mission_bt_runner running with bt_xml:=task_loop.xml ?')
        return 1
    task_id = time.strftime('cli-%H%M%S')
    node.watch = task_id
    node.send(task_id, action, duration, points)
    node.wait_until_finished([task_id])
    return 0


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    rclpy.init()
    node = TaskSender()
    try:
        if args:
            return once(node, args)
        interactive(node)
        return 0
    except ValueError as e:
        print(e)
        return 1
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
