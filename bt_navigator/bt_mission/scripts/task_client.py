#!/usr/bin/env python3
"""
task_server 에 작업을 액션(execute_task, bt_mission_msgs/action/ExecuteTask)으로 맡긴다.

  # 대화형
  ros2 run bt_mission task_client.py
  task> inspect 0,-1 1.5,-1,90       작업 추가. 하고 있는 작업이 있으면 끝난 뒤에 보낸다
  task> !go -2,-1,0                  지금 작업을 멈추고(선점) 바로 보낸다
  task> cancel                       지금 작업을 취소하고, 기다리는 작업도 비운다

  # 작업 하나만 보내고, 끝날 때까지 기다렸다가 종료 (종료 코드 0 = SUCCEEDED)
  ros2 run bt_mission task_client.py inspect 0,-1 1.5,-1,90

작업 한 줄의 형식은 task_sender.py 와 같다: <action>[:duration] x,y[,yaw_deg] [x,y[,yaw_deg] ...]
Ctrl-C 를 누르면 지금 작업을 취소하고 끝낸다.
"""

from collections import deque
import math
import select
import signal
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from bt_mission_msgs.action import ExecuteTask
from bt_mission_msgs.msg import Task
import tf2_ros

# 작업 한 줄 해석과 yaw 채우기는 같은 폴더의 task_sender.py (토픽 버전) 와 같다
from task_sender import fill_yaws, parse_line

HELP = '''작업: <action>[:duration] x,y[,yaw_deg] [x,y[,yaw_deg] ...]   (map frame)
  action: go | inspect | scan | look_around. 점마다 멈춰서 도착을 알리고 다음 점으로, 작업은 마지막 점에서
  예) inspect 0,-1 1.5,-1,90    scan 1.5,1.5 -1,1.5,180    look_around -1,1.5,180    go -2,-1,0
  !<작업>: 지금 작업을 멈추고 바로 보냄    cancel: 지금 작업 취소 + 대기 비우기
  help: 이 도움말, quit 또는 Ctrl-D: 끝내기 (남은 작업이 끝날 때까지 기다림), Ctrl-C: 취소하고 끝내기'''

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}

# Ctrl-C 는 플래그로만 받고, 기다리는 루프에서 확인한다. rclpy 기본 처리는 ROS 를 먼저 종료시켜서
# 취소 요청을 보낼 수 없고, 콜백 도중에 KeyboardInterrupt 가 나면 rclpy 내부가 꼬일 수 있다.
interrupted = False


def on_sigint(*_):
    global interrupted
    interrupted = True


class TaskClient(Node):

    def __init__(self):
        super().__init__('task_client')
        self.client = ActionClient(self, ExecuteTask, 'execute_task')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.queue = deque()        # 아직 보내지 않은 작업 (task_id, Task)
        self.active = None          # 마지막으로 보낸 작업 id (끝나면 None)
        self.handles = {}           # 작업 id → goal handle
        self.last_xy = None         # 직전 작업의 마지막 점 (yaw 채우기용, 처음엔 로봇 위치)
        self.last_running = {}
        self.last_reached = {}      # 작업 id → 도착을 출력한 점 수
        self.sent = {}              # 작업 id → 보낸 Task (도착한 점 좌표 출력용)
        self.results = {}           # 작업 id → (상태, 메시지)
        self.count = 0

    # ---- 기다리기 (Ctrl-C 확인하면서 ROS 콜백 처리) -----------------------------------------

    def spin_for(self, seconds):
        rclpy.spin_once(self, timeout_sec=seconds)
        if interrupted:
            raise KeyboardInterrupt

    def wait_for_server(self, timeout=10.0):
        deadline = time.time() + timeout
        while not self.client.server_is_ready():
            if time.time() > deadline:
                return False
            self.spin_for(0.1)
        return True

    def read_line(self, prompt):
        """stdin 에서 한 줄 읽기. 기다리는 동안에도 feedback, result 를 받는다."""
        print(prompt, end='', flush=True)
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                line = sys.stdin.readline()
                if not line:
                    raise EOFError
                return line
            self.spin_for(0.0)

    def read_start_pose(self, timeout=1.0):
        """로봇 위치를 한 번 읽고 TF 구독을 끊는다 (계속 받으면 feedback 처리가 밀린다)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', Time()).transform
                self.last_xy = (t.translation.x, t.translation.y)
                break
            except tf2_ros.TransformException:
                self.spin_for(0.05)
        self.tf_listener.unregister()

    def wait_until_idle(self):
        while self.active is not None or self.queue:
            self.spin_for(0.1)

    # ---- 작업 보내기 ---------------------------------------------------------------------------

    def make_task(self, action, duration, points):
        self.count += 1
        task = Task(id=f't{self.count}', action=action, duration=duration)
        start = self.last_xy or (points[0][0], points[0][1])
        stamp = self.get_clock().now().to_msg()
        for x, y, yaw in fill_yaws(start, points):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            task.waypoints.append(pose)
        self.last_xy = (points[-1][0], points[-1][1])
        return task

    def add(self, task, now=False):
        """now 면 바로 보낸다 (작업 중이면 서버가 그 작업을 멈춘다). 아니면 차례를 기다린다."""
        if now or self.active is None:
            self.send(task)
        else:
            self.queue.append(task)
            print(f'  .. {task.id} queued ({len(self.queue)} waiting)', flush=True)

    def send(self, task):
        route = ' -> '.join(f'({p.pose.position.x:g},{p.pose.position.y:g})'
                            for p in task.waypoints)
        print(f'  -> {task.id}: {task.action} {route}', flush=True)
        self.active = task.id
        self.sent[task.id] = task
        future = self.client.send_goal_async(
            ExecuteTask.Goal(task=task),
            feedback_callback=lambda msg: self.on_feedback(msg.feedback))
        future.add_done_callback(lambda f: self.on_goal_response(task.id, f))

    def send_next(self):
        if self.active is None and self.queue:
            self.send(self.queue.popleft())

    def cancel(self):
        self.queue.clear()
        handle = self.handles.get(self.active)
        if handle is not None:
            print(f'  .. canceling {self.active}', flush=True)
            handle.cancel_goal_async()

    # ---- 응답 처리 ----------------------------------------------------------------------------

    def on_goal_response(self, task_id, future):
        handle = future.result()
        if not handle.accepted:
            self.finished(task_id, 'REJECTED', 'rejected by task_server')
            return
        self.handles[task_id] = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(task_id, f))

    def print_arrivals(self, task_id, reached, when=''):
        """지난번 출력 뒤로 새로 도착한 점을 하나씩 출력한다."""
        task = self.sent.get(task_id)
        total = len(task.waypoints) if task is not None else reached
        for i in range(self.last_reached.get(task_id, 0), reached):
            where = ''
            if task is not None and i < total:
                p = task.waypoints[i].pose.position
                where = f' ({p.x:g},{p.y:g})'
            print(f'  .. {task_id}: arrived {i + 1}/{total}{where}{when}', flush=True)
        self.last_reached[task_id] = max(self.last_reached.get(task_id, 0), reached)

    def on_feedback(self, feedback):
        task_id = feedback.task_id
        self.print_arrivals(task_id, feedback.waypoints_reached, f' ({feedback.elapsed:.1f} s)')
        if feedback.running and feedback.running != self.last_running.get(task_id):
            print(f'  .. {task_id}: {feedback.running} ({feedback.elapsed:.1f} s)', flush=True)
        self.last_running[task_id] = feedback.running

    def on_result(self, task_id, future):
        response = future.result()
        self.finished(task_id, STATUS.get(response.status, str(response.status)),
                      response.result.message)

    def finished(self, task_id, state, message):
        # 마지막 점에 도착하면서 끝나는 작업(go, scan)은 그 도착을 담은 feedback 없이 결과가 온다.
        # SUCCEEDED 면 모든 점에 도착한 것이므로 남은 도착을 여기서 출력한다
        if state == 'SUCCEEDED' and task_id in self.sent:
            self.print_arrivals(task_id, len(self.sent[task_id].waypoints))
        print(f'  <- {task_id} {state}: {message}', flush=True)
        self.results[task_id] = (state, message)
        self.handles.pop(task_id, None)
        if self.active == task_id:
            self.active = None
            self.send_next()


def interactive(node):
    print(HELP)
    while True:
        try:
            line = node.read_line('task> ').split('#')[0].strip()
        except EOFError:                          # Ctrl-D, 또는 파이프 입력이 끝남
            break
        if not line:
            continue
        if line in ('quit', 'exit'):
            break
        if line == 'help':
            print(HELP)
            continue
        if line == 'cancel':
            node.cancel()
            continue
        now = line.startswith('!')
        try:
            action, duration, points = parse_line(line.lstrip('!'))
        except ValueError as e:
            print(f'  ! {e}')
            continue
        node.add(node.make_task(action, duration, points), now=now)
    if node.active is not None or node.queue:
        print(f'waiting for {len(node.queue) + 1} task(s) to finish (Ctrl-C to cancel)')
        node.wait_until_idle()
    return 0


def once(node, args):
    action, duration, points = parse_line(' '.join(args))
    task = node.make_task(action, duration, points)
    node.add(task)
    node.wait_until_idle()
    return 0 if node.results.get(task.id, ('',))[0] == 'SUCCEEDED' else 1


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, on_sigint)
    node = TaskClient()
    try:
        if not node.wait_for_server():
            print('execute_task action server is not up. Is task_server running?')
            return 1
        node.read_start_pose()
        return once(node, args) if args else interactive(node)
    except ValueError as e:
        print(e)
        return 1
    except KeyboardInterrupt:
        # 지금 작업을 취소하고 결과(CANCELED)가 올 때까지 잠깐 기다린다
        global interrupted
        interrupted = False
        node.cancel()
        deadline = time.time() + 5.0
        while node.active is not None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
