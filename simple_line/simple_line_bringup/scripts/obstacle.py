#!/usr/bin/env python3
"""
Gazebo 에 장애물(빨간 원기둥)을 놓고 치운다. stop & go 를 직접 시험할 때 쓴다.

  ros2 run simple_line_bringup obstacle.py put 0,-1            # (0,-1) 에 놓는다 (이름은 obstacle_<놓은 시각>)
  ros2 run simple_line_bringup obstacle.py put 0,-1 --radius 0.2
  ros2 run simple_line_bringup obstacle.py remove              # 이 도구로 놓은 장애물(obstacle_*)을 모두 치운다
  ros2 run simple_line_bringup obstacle.py remove obstacle_1
  ros2 run simple_line_bringup obstacle.py click               # RViz 의 Publish Point 로 찍은 곳에 놓고,
                                                               # 놓은 장애물을 다시 찍으면 치운다 (Ctrl-C 로 끝)

좌표는 map 좌표다. simple_line.launch.py 의 시뮬레이션은 map 과 Gazebo world 좌표가 같다.
gazebo_ros 의 /spawn_entity, /delete_entity, /get_model_list 서비스를 쓴다.
지운 모델과 같은 이름으로 다시 놓으면 gzserver 가 죽는 경우가 있어서(Gazebo 11) 이름을 다시 쓰지 않는다.
"""

import argparse
import math
import sys
import time

from gazebo_msgs.srv import DeleteEntity, GetModelList, SpawnEntity
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

PREFIX = 'obstacle_'
HEIGHT = 0.5  # [m] 레이저(높이 약 0.17m)에 보이게

SDF = """<sdf version="1.6">
  <model name="obstacle">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        <material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


class Obstacles(Node):

    def __init__(self):
        super().__init__('obstacle_tool')
        self.spawn_cli = self.create_client(SpawnEntity, 'spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, 'delete_entity')
        self.list_cli = self.create_client(GetModelList, 'get_model_list')
        self.placed = {}  # click 모드에서 놓은 것: 이름 -> (x, y, radius)
        self.used = set()  # 이 실행에서 쓴 이름 (지운 것도 다시 쓰지 않는다)

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'service {client.srv_name} not available (is Gazebo running?)')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is None:
            raise RuntimeError(f'service {client.srv_name} did not answer')
        return future.result()

    def names(self):
        return [n for n in self.call(self.list_cli, GetModelList.Request()).model_names
                if n.startswith(PREFIX)]

    def put(self, x, y, radius):
        used = set(self.names())
        stamp = time.strftime('%H%M%S')
        name = next(n for n in [f'{PREFIX}{stamp}'] + [f'{PREFIX}{stamp}_{i}' for i in range(2, 100)]
                    if n not in used and n not in self.used)
        self.used.add(name)
        req = SpawnEntity.Request()
        req.name = name
        req.xml = SDF.format(r=radius, h=HEIGHT)
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = HEIGHT / 2.0
        req.reference_frame = 'world'
        res = self.call(self.spawn_cli, req)
        if not res.success:
            raise RuntimeError(res.status_message)
        self.get_logger().info(f'put {name} at ({x:.2f}, {y:.2f}), radius {radius:.2f} m')
        return name

    def remove(self, name):
        req = DeleteEntity.Request()
        req.name = name
        res = self.call(self.delete_cli, req)
        if not res.success:
            raise RuntimeError(res.status_message)
        self.placed.pop(name, None)
        self.get_logger().info(f'removed {name}')

    def on_click(self, msg, radius):
        x, y = msg.point.x, msg.point.y
        if msg.header.frame_id not in ('map', '/map'):
            self.get_logger().warn(f'point in frame {msg.header.frame_id}, used as map coordinates')
        try:
            for name, (ox, oy, r) in list(self.placed.items()):
                if math.hypot(x - ox, y - oy) <= r + 0.15:
                    self.remove(name)
                    return
            name = self.put(x, y, radius)
            self.placed[name] = (x, y, radius)
        except RuntimeError as e:
            self.get_logger().error(str(e))


def parse_xy(text):
    try:
        x, y = (float(v) for v in text.split(','))
    except ValueError:
        raise argparse.ArgumentTypeError(f"'{text}' is not x,y (e.g. 0,-1)")
    return x, y


def main():
    parser = argparse.ArgumentParser(description='Gazebo 에 장애물을 놓고 치운다 (stop & go 시험용)')
    sub = parser.add_subparsers(dest='command', required=True)
    p_put = sub.add_parser('put', help='x,y 에 원기둥을 놓는다')
    p_put.add_argument('xy', type=parse_xy, help='map 좌표 x,y [m]')
    p_put.add_argument('--radius', type=float, default=0.15, help='반지름 [m] (기본 0.15)')
    p_remove = sub.add_parser('remove', help='치운다 (이름을 안 주면 obstacle_* 모두)')
    p_remove.add_argument('name', nargs='?')
    p_click = sub.add_parser('click', help='RViz Publish Point 로 놓고 치운다')
    p_click.add_argument('--radius', type=float, default=0.15, help='반지름 [m] (기본 0.15)')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Obstacles()
    try:
        if args.command == 'put':
            node.put(*args.xy, args.radius)
        elif args.command == 'remove':
            names = [args.name] if args.name else node.names()
            if not names:
                node.get_logger().info('no obstacles to remove')
            for name in names:
                node.remove(name)
        else:
            # 서비스 응답을 기다리는 일은 콜백 밖(이 루프)에서 한다
            clicks = []
            node.create_subscription(PointStamped, 'clicked_point', clicks.append, 10)
            node.get_logger().info(
                'RViz 상단의 Publish Point 로 바닥을 찍으면 장애물을 놓고, 놓은 장애물을 다시 찍으면 치운다')
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                while clicks:
                    node.on_click(clicks.pop(0), args.radius)
    except RuntimeError as e:
        node.get_logger().error(str(e))
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
