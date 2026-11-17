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

  # 시나리오 파일: 한 줄에 작업 하나 (대화형과 같은 형식, wait 포함). 차례로 하고, 하나라도 실패하면
  # 남은 줄은 하지 않는다 (--keep-going 이면 계속). 끝에 작업마다 결과와 걸린 시간. 종료 코드 0 = 모두 SUCCEEDED
  ros2 run bt_mission task_client.py --file $(ros2 pkg prefix bt_mission)/share/bt_mission/scenarios/s1_patrol_dock.txt
  ros2 run bt_mission task_client.py < my_scenario.txt          # 파이프로 넣어도 같다

  task> wait 10                      작업 사이에 10 초 기다린다 (설비 작업 시간 흉내. 앞 작업이 끝난 뒤부터 센다)

작업 한 줄의 형식: <action>[:option] x,y[,yaw_deg] [x,y[,yaw_deg] ...]
  - option: inspect 는 작업 시간 [s] (inspect:5), dock 과 undock_precise 는 config/docking.yaml 의 dock 이름
    (dock:dock_a, 비우면 처음 보이는 marker 의 dock)
  - 좌표는 map frame. 로봇은 점마다 멈춰서 위치와 방향을 맞추고(도착) 다음 점으로 간다.
    작업(inspect 의 점검, look_around 의 회전, dock 의 도킹)은 마지막 점에서 한다.
    dock 의 마지막 점은 dock 앞에서 marker 가 보이는 자리(staging)다
  - undock, undock_precise 는 점 없이 쓴다: 그 자리에서 dock 을 빠져나온다 (undock: 곧게 뒤로,
    undock_precise[:dock]: marker 를 보며 dock 축을 따라 물러나 정해진 자리에 선다)
  - yaw 는 그 점에 도착했을 때 볼 방향 [deg]. 생략하면 중간 점은 다음 점 쪽,
    마지막 점은 직전 점에서 온 방향으로 채운다.
    작업의 첫 점은 직전 작업의 마지막 점(첫 작업이면 로봇 현재 위치)에서 온 것으로 본다

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

ACTIONS = (Task.ACTION_GO, Task.ACTION_INSPECT, Task.ACTION_SCAN, Task.ACTION_LOOK_AROUND,
           Task.ACTION_DOCK, Task.ACTION_UNDOCK, Task.ACTION_UNDOCK_PRECISE)
DEFAULT_DURATION = 3.0

HELP = '''작업: <action>[:option] x,y[,yaw_deg] [x,y[,yaw_deg] ...]   (map frame)
  action: go | inspect | scan | look_around | dock | undock | undock_precise. 점마다 멈춰서 도착을 알리고 다음 점으로, 작업은 마지막 점에서
  예) inspect:5 0,-1 1.5,-1,90    scan 1.5,1.5 -1,1.5,180    look_around -1,1.5,180    go -2,-1,0
      dock:dock_a 3.85,1.0,0 (마지막 점 = dock 앞 staging 자리, dock_a = docking.yaml 의 dock)
      undock (점 없음, 곧게 뒤로)    undock_precise:dock_a (점 없음, marker 를 보며 물러남)
  wait <초>: 앞 작업이 끝난 뒤 그만큼 기다렸다가 다음 작업 (설비 작업 시간 흉내)
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


class Wait:
    """작업 사이의 기다림 (wait <초>)"""

    def __init__(self, seconds):
        self.seconds = seconds


def parse_wait(text):
    """'wait 10' -> 10.0. wait 줄이 아니면 None"""
    tokens = text.split()
    if not tokens or tokens[0] != 'wait':
        return None
    if len(tokens) != 2:
        raise ValueError('wait needs one number of seconds: wait 10')
    try:
        seconds = float(tokens[1])
    except ValueError:
        raise ValueError(f"wait needs a number of seconds, not '{tokens[1]}': wait 10") from None
    if seconds < 0:
        raise ValueError('wait needs a positive number of seconds')
    return seconds


def parse_line(text):
    """'inspect:5 0,-1 1.5,-1,90' -> ('inspect', 5.0, '', [(0, -1, None), (1.5, -1, 90)])
    'dock:dock_a 3.5,1.0,0' -> ('dock', 3.0, 'dock_a', [(3.5, 1.0, 0)])"""
    tokens = text.split()
    if not tokens:
        raise ValueError('need an action')
    action, _, option = tokens[0].partition(':')
    if action not in ACTIONS:
        raise ValueError(f"unknown action '{action}' (one of {', '.join(ACTIONS)})")
    if len(tokens) < 2 and action not in (Task.ACTION_UNDOCK, Task.ACTION_UNDOCK_PRECISE):
        raise ValueError('need at least one point')
    duration, target = DEFAULT_DURATION, ''
    if action in (Task.ACTION_DOCK, Task.ACTION_UNDOCK_PRECISE):
        target = option
    elif option:
        try:
            duration = float(option)
        except ValueError:
            raise ValueError(f"duration '{option}' is not a number (e.g. inspect:5)") from None
    points = []
    for token in tokens[1:]:
        try:
            values = [float(v) for v in token.split(',')]
        except ValueError:
            values = []
        if len(values) not in (2, 3):
            raise ValueError(f"'{token}' is not x,y or x,y,yaw_deg")
        points.append((values[0], values[1], values[2] if len(values) == 3 else None))
    return action, duration, target, points


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
        self.started = {}           # 작업 id → 보낸 시각 (요약의 걸린 시간)
        self.durations = {}         # 작업 id → 걸린 시간 [s]
        self.stop_on_failure = False   # 시나리오 파일: 하나가 실패하면 남은 것을 하지 않는다
        self.skipped = 0            # 그래서 하지 않은 줄 수
        self.failed_lines = 0       # 읽지 못한 줄 수 (형식이 틀림)
        self.wait_until = None      # wait 중이면 끝나는 시각
        self.create_timer(0.1, self.check_wait)

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

    def make_task(self, action, duration, points, target=''):
        self.count += 1
        task = Task(id=f't{self.count}', action=action, duration=duration, target=target)
        if not points:                      # undock: 그 자리에서
            return task
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
        """now 면 바로 보낸다 (작업 중이면 서버가 그 작업을 멈춘다). 아니면 차례를 기다린다.
        task 는 Task 또는 Wait"""
        if now or self.active is None:
            self.start(task)
        else:
            self.queue.append(task)
            what = f'wait {task.seconds:g} s' if isinstance(task, Wait) else task.id
            print(f'  .. {what} queued ({len(self.queue)} waiting)', flush=True)

    def start(self, item):
        if isinstance(item, Wait):
            self.active = 'wait'
            self.wait_until = time.time() + item.seconds
            print(f'  .. waiting {item.seconds:g} s', flush=True)
        else:
            self.send(item)

    def check_wait(self):
        if self.active == 'wait' and time.time() >= self.wait_until:
            print('  .. wait done', flush=True)
            self.active = None
            self.send_next()

    def send(self, task):
        route = ' -> '.join(f'({p.pose.position.x:g},{p.pose.position.y:g})'
                            for p in task.waypoints) or '(here)'
        target = f' [{task.target}]' if task.target else ''
        print(f'  -> {task.id}: {task.action}{target} {route}', flush=True)
        self.active = task.id
        self.sent[task.id] = task
        self.started[task.id] = time.time()
        future = self.client.send_goal_async(
            ExecuteTask.Goal(task=task),
            feedback_callback=lambda msg: self.on_feedback(msg.feedback))
        future.add_done_callback(lambda f: self.on_goal_response(task.id, f))

    def send_next(self):
        if self.active is None and self.queue:
            self.start(self.queue.popleft())

    def cancel(self):
        self.queue.clear()
        if self.active == 'wait':
            print('  .. wait canceled', flush=True)
            self.active = None
            return
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
            # bt_status: 트리(루트)의 상태 (실행 중이므로 RUNNING), running: 지금 RUNNING 인 액션 노드
            print(f'  .. {task_id}: {feedback.bt_status} {feedback.running} '
                  f'({feedback.elapsed:.1f} s)', flush=True)
        self.last_running[task_id] = feedback.running

    def on_result(self, task_id, future):
        response = future.result()
        self.finished(task_id, STATUS.get(response.status, str(response.status)),
                      response.result.message, response.result.bt_status)

    def finished(self, task_id, state, message, bt_status=''):
        # 마지막 점에 도착하면서 끝나는 작업(go, scan)은 그 도착을 담은 feedback 없이 결과가 온다.
        # SUCCEEDED 면 모든 점에 도착한 것이므로 남은 도착을 여기서 출력한다
        if state == 'SUCCEEDED' and task_id in self.sent:
            self.print_arrivals(task_id, len(self.sent[task_id].waypoints))
        # state: 액션 goal 의 상태, bt_status: 트리(루트)가 마지막으로 돌려준 상태
        print(f'  <- {task_id} {state}: {message}' + (f' [BT {bt_status}]' if bt_status else ''),
              flush=True)
        self.results[task_id] = (state, message)
        if task_id in self.started:
            self.durations[task_id] = time.time() - self.started[task_id]
        self.handles.pop(task_id, None)
        if state != 'SUCCEEDED' and self.stop_on_failure and self.queue:
            self.skipped += len(self.queue)
            print(f'  !! {task_id} {state}: stopping here, {len(self.queue)} line(s) not run '
                  '(--keep-going to go on)', flush=True)
            self.queue.clear()
        if self.active == task_id:
            self.active = None
            self.send_next()


def handle_line(node, line):
    """한 줄 처리. False 면 그만 읽는다 (quit)"""
    line = line.split('#')[0].strip()
    if not line:
        return True
    if line in ('quit', 'exit'):
        return False
    if line == 'help':
        print(HELP)
        return True
    if line == 'cancel':
        node.cancel()
        return True
    now = line.startswith('!')
    try:
        seconds = parse_wait(line.lstrip('!'))
        if seconds is not None:
            node.add(Wait(seconds))
            return True
        action, duration, target, points = parse_line(line.lstrip('!'))
    except ValueError as e:
        print(f'  ! {e}')
        node.failed_lines += 1
        return True
    node.add(node.make_task(action, duration, points, target), now=now)
    return True


def summary(node):
    """시나리오 파일의 끝: 작업마다 결과와 걸린 시간. 모두 SUCCEEDED 이면 0"""
    print('\nsummary:')
    ok = True
    for task_id, task in node.sent.items():
        state, message = node.results.get(task_id, ('NOT FINISHED', ''))
        target = f':{task.target}' if task.target else ''
        took = node.durations.get(task_id)
        print(f"  {task_id:4s} {task.action + target:22s} {state:10s} "
              f"{f'{took:6.1f} s' if took is not None else '':8s}  {message}")
        ok = ok and state == 'SUCCEEDED'
    bad_lines = node.failed_lines
    if node.skipped:
        print(f'  ({node.skipped} line(s) not run after a failure)')
    if bad_lines:
        print(f'  ({bad_lines} line(s) could not be read)')
    passed = ok and not node.skipped and not bad_lines
    print('  ALL SUCCEEDED' if passed else '  FAILED')
    return 0 if passed else 1


def interactive(node):
    """키보드 (대화형) 또는 파이프 (시나리오). 파이프면 읽은 줄을 보여 주고, 실패하면 멈추고, 끝에 요약"""
    piped = not sys.stdin.isatty()
    if not piped:
        print(HELP)
    while True:
        try:
            line = node.read_line('' if piped else 'task> ')
        except EOFError:                          # Ctrl-D, 또는 파이프 입력이 끝남
            break
        if piped and line.split('#')[0].strip():
            print(f'task> {line.rstrip()}', flush=True)
        if not handle_line(node, line):
            break
    if node.active is not None or node.queue:
        print(f'waiting for {len(node.queue) + 1} item(s) to finish (Ctrl-C to cancel)')
        node.wait_until_idle()
    return summary(node) if piped else 0


def from_file(node, path):
    """시나리오 파일의 줄을 모두 넣고, 끝날 때까지 기다린 뒤 요약"""
    with open(path) as f:
        lines = f.readlines()
    print(f'scenario {path}')
    for line in lines:
        if line.split('#')[0].strip():
            print(f'task> {line.rstrip()}', flush=True)
        if not handle_line(node, line):
            break
    node.wait_until_idle()
    return summary(node)


def once(node, args):
    action, duration, target, points = parse_line(' '.join(args))
    task = node.make_task(action, duration, points, target)
    node.add(task)
    node.wait_until_idle()
    return 0 if node.results.get(task.id, ('',))[0] == 'SUCCEEDED' else 1


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    keep_going = '--keep-going' in args
    args = [a for a in args if a != '--keep-going']
    scenario = None
    if args and args[0] in ('-f', '--file'):
        if len(args) != 2:
            print('usage: task_client.py --file <scenario.txt> [--keep-going]')
            return 1
        scenario = args[1]
        args = []
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, on_sigint)
    node = TaskClient()
    try:
        if not node.wait_for_server():
            print('execute_task action server is not up. Is task_server running?')
            return 1
        node.read_start_pose()
        node.stop_on_failure = (scenario is not None or not sys.stdin.isatty()) and not keep_going
        if scenario is not None:
            return from_file(node, scenario)
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
