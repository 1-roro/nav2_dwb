#!/usr/bin/env python3
"""
시나리오 실행기 (Gazebo 실습용).

  ros2 run line_tutorial_bringup send_scenario.py S2_behind
  ros2 run line_tutorial_bringup send_scenario.py S2_behind --bt replan
  ros2 run line_tutorial_bringup send_scenario.py --list

1. 출발점으로 이동
   Nav2 기본 플러그인(NavFn planner + RPP controller)으로 로봇을 시나리오 출발 자세까지 옮긴다.
   이 이동은 튜토리얼 플러그인과 상관없고, 이때만 goal checker 허용오차를 좁혀서
   출발 자세를 맞춘다 (끝나면 원래 값으로 되돌린다).
2. 실험
   NavigateToPose 로 시나리오 goal 을 보낸다. bt_navigator 가 GridBased(직선 planner)와
   FollowPath(튜토리얼 controller)를 현재 step 파라미터 그대로 실행한다.
   자세한 오차는 cte_monitor 가 출력한다.

시나리오 좌표는 line_room 월드 기준이고, line_tutorial_sim/kinematic_sim.hpp 의
defaultScenarios() 와 같다. 바꾸면 같이 고칠 것.
"""

import argparse
import math
import os
import sys
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

D = math.pi / 180
A = (-2.0, -1.0, 0.0)
B = (1.0, -1.0, 0.0)
SCENARIOS = {
    'S1_straight': (A, (1.0, -1.0, 0.0), 'A -> B: 3 m straight ahead along map +x'),
    'S2_behind': (B, (-2.0, -1.0, 180 * D), 'B -> A: goal straight behind the robot'),
    'S3_pillar': (A, (1.0, 1.0, math.atan2(2.0, 3.0)),
                  'A -> (1, 1): the pillar sits in the middle of the line'),
    'S4_goal_yaw': (A, (1.0, -1.0, 90 * D), 'A -> B, but the goal yaw is +90 deg'),
    'S5_offset': (B, (1.3, 2.0, math.atan2(3.0, 0.3)),
                  'B -> (1.3, 2): the line is 84 deg to the left of the robot'),
}
BT_FILES = {'once': 'plan_once.xml', 'replan': 'replan_1hz.xml'}
CHECKER = 'general_goal_checker'
SETUP_TOLERANCE = {'xy_goal_tolerance': 0.08, 'yaw_goal_tolerance': 0.05}
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}


def pose_stamped(x, y, yaw, stamp):
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = stamp
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.z = math.sin(yaw / 2)
    ps.pose.orientation.w = math.cos(yaw / 2)
    return ps


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class ScenarioRunner(Node):

    def __init__(self):
        super().__init__('send_scenario')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_params = self.create_client(GetParameters, '/controller_server/get_parameters')
        self.set_params = self.create_client(SetParameters, '/controller_server/set_parameters')

    # ---------------------------------------------------------------- helpers

    def wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def robot_pose(self):
        end = time.time() + 5.0
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            except tf2_ros.TransformException:
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return t.x, t.y, yaw
        raise RuntimeError('no TF map -> base_link (is Nav2 running and localized?)')

    def tolerances(self):
        names = [f'{CHECKER}.{k}' for k in SETUP_TOLERANCE]
        res = self.wait(self.get_params.call_async(GetParameters.Request(names=names)), 5.0)
        return {k: v.double_value for k, v in zip(SETUP_TOLERANCE, res.values)}

    def set_tolerances(self, values):
        params = [Parameter(name=f'{CHECKER}.{k}',
                            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                 double_value=float(v)))
                  for k, v in values.items()]
        self.wait(self.set_params.call_async(SetParameters.Request(parameters=params)), 5.0)

    def run_action(self, client, goal, timeout):
        handle = self.wait(client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            return 'REJECTED'
        result = self.wait(handle.get_result_async(), timeout)
        if result is None:
            self.wait(handle.cancel_goal_async(), 5.0)
            return 'TIMEOUT'
        return STATUS.get(result.status, str(result.status))

    # ---------------------------------------------------------------- steps

    def move_to_start(self, start):
        x, y, yaw = self.robot_pose()
        if math.hypot(x - start[0], y - start[1]) < 0.1 and abs(angle_diff(yaw, start[2])) < 0.06:
            print('  already at the start pose')
            return True
        print(f'  moving to the start pose ({start[0]:.2f}, {start[1]:.2f}, '
              f'{math.degrees(start[2]):.0f} deg) with NavFn + RPP ...')
        saved = self.tolerances()
        self.set_tolerances(SETUP_TOLERANCE)
        try:
            plan_goal = ComputePathToPose.Goal(
                goal=pose_stamped(*start, self.get_clock().now().to_msg()), planner_id='NavFn')
            handle = self.wait(self.plan_client.send_goal_async(plan_goal), 10.0)
            if handle is None or not handle.accepted:
                print('  planning to the start pose was rejected')
                return False
            result = self.wait(handle.get_result_async(), 10.0)
            if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
                print('  planning to the start pose failed')
                return False
            follow = FollowPath.Goal(path=result.result.path, controller_id='RPP')
            status = self.run_action(self.follow_client, follow, 120.0)
            print(f'  start pose: {status}')
            return status == 'SUCCEEDED'
        finally:
            self.set_tolerances(saved)

    def run(self, name, bt, skip_setup):
        start, goal, desc = SCENARIOS[name]
        for client in (self.plan_client, self.follow_client, self.nav_client):
            if not client.wait_for_server(timeout_sec=30.0):
                raise RuntimeError('Nav2 action servers are not up')
        for srv in (self.get_params, self.set_params):
            if not srv.wait_for_service(timeout_sec=10.0):
                raise RuntimeError('controller_server parameter services are not up')

        print(f'[{name}] {desc}')
        if not skip_setup and not self.move_to_start(start):
            return 1
        time.sleep(0.5)

        bt_path = ''
        if bt:
            share = get_package_share_directory('line_tutorial_bringup')
            bt_path = os.path.join(share, 'behavior_trees', BT_FILES[bt])
        print(f'  sending goal ({goal[0]:.2f}, {goal[1]:.2f}, {math.degrees(goal[2]):.0f} deg)'
              f'{" with " + BT_FILES[bt] if bt else ""} ...')
        t0 = time.time()
        nav_goal = NavigateToPose.Goal(
            pose=pose_stamped(*goal, self.get_clock().now().to_msg()), behavior_tree=bt_path)
        status = self.run_action(self.nav_client, nav_goal, 120.0)
        x, y, yaw = self.robot_pose()
        print(f'  result: {status} after {time.time() - t0:.1f} s, final error '
              f'xy {math.hypot(x - goal[0], y - goal[1]):.2f} m, '
              f'yaw {abs(math.degrees(angle_diff(yaw, goal[2]))):.1f} deg')
        return 0 if status == 'SUCCEEDED' else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('scenario', nargs='?', choices=sorted(SCENARIOS))
    parser.add_argument('--bt', choices=sorted(BT_FILES),
                        help='override the behavior tree for this goal only')
    parser.add_argument('--no-setup', action='store_true',
                        help='do not move to the start pose first')
    parser.add_argument('--list', action='store_true', help='list the scenarios')
    args, _ = parser.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    if args.list or not args.scenario:
        for name, (start, goal, desc) in SCENARIOS.items():
            print(f'{name:<12} start ({start[0]:5.2f}, {start[1]:5.2f}, '
                  f'{math.degrees(start[2]):4.0f}) goal ({goal[0]:5.2f}, {goal[1]:5.2f}, '
                  f'{math.degrees(goal[2]):4.0f})  {desc}')
        return 0

    rclpy.init()
    node = ScenarioRunner()
    try:
        return node.run(args.scenario, args.bt, args.no_setup)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
