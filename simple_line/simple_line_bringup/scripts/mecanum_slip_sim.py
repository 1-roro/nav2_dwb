#!/usr/bin/env python3
"""
시뮬레이션에서 메카넘 바퀴의 미끄러짐을 흉내 낸다 (simple_line.launch.py robot:=holonomic mecanum_slip:=True).

gazebo_ros_planar_move 는 명령한 속도대로 몸체를 정확히 움직이고, 그 참값을 odom 으로 낸다. 실제 메카넘 로봇은
  - 바퀴는 명령대로 돌지만 몸체는 덜 간다 (롤러가 미끄러진다. 옆으로 갈 때 특히: 흔히 80 ~ 95 %)
  - 옆으로 가면 바닥과 하중에 따라 조금씩 돈다
  - 바퀴 odom 은 "바퀴가 돈 만큼" 을 적분하므로 미끄러진 만큼 틀린다
이 노드가 그 사이에 들어간다:

  velocity_smoother --cmd_vel--> [이 노드] --cmd_vel_body (미끄러진 속도)--> planar_move (몸체)
                                     └-- /odom, TF odom -> base_footprint (바퀴 odom: 명령을 적분)
  planar_move 의 참값 odom 은 odom_truth 로 옮긴다 (처음 자리를 잡을 때만 쓴다)

파라미터 (가정한 값. 실제 로봇에서 재서 바꾼다, ARUCO_DOCKING.md 6장)
  scale_x 0.98, scale_y 0.90, scale_wz 0.96   몸체가 실제로 가는 비율
  yaw_per_lateral 0.04                         옆으로 1 m 갈 때 도는 각 [rad]
  noise 0.02                                   속도에 비례하는 잡음 (표준편차 비율)
"""

import math
import random

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class MecanumSlipSim(Node):

    def __init__(self):
        super().__init__('mecanum_slip_sim')
        self.sx = self.declare_parameter('scale_x', 0.98).value
        self.sy = self.declare_parameter('scale_y', 0.90).value
        self.sw = self.declare_parameter('scale_wz', 0.96).value
        self.yaw_per_lateral = self.declare_parameter('yaw_per_lateral', 0.04).value
        self.noise = self.declare_parameter('noise', 0.02).value
        rate = self.declare_parameter('rate', 50.0).value
        self.cmd = Twist()
        self.cmd_time = None
        self.pose = None                      # 바퀴 odom (x, y, yaw)
        self.last = None
        self.tf = TransformBroadcaster(self)
        self.body_pub = self.create_publisher(Twist, 'cmd_vel_body', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Odometry, 'odom_truth', self.on_truth, 10)
        self.create_timer(1.0 / rate, self.step)
        self.get_logger().info(
            f'mecanum slip: body moves x {self.sx:.2f}, y {self.sy:.2f}, wz {self.sw:.2f} of the command, '
            f'{self.yaw_per_lateral:.3f} rad per lateral m, noise {self.noise:.0%}')

    def on_cmd(self, msg):
        self.cmd = msg
        self.cmd_time = self.get_clock().now()

    def on_truth(self, msg):
        if self.pose is None:                 # 처음에만: 바퀴 odom 을 참값에서 시작한다 (그 뒤로는 따로 흐른다)
            q = msg.pose.pose.orientation
            self.pose = [msg.pose.pose.position.x, msg.pose.pose.position.y,
                         2.0 * math.atan2(q.z, q.w)]

    def step(self):
        now = self.get_clock().now()
        if self.pose is None:
            return
        cmd = self.cmd
        if self.cmd_time is None or (now - self.cmd_time).nanoseconds * 1e-9 > 0.5:
            cmd = Twist()                     # 명령이 끊기면 선다 (모터 드라이버의 watchdog)
        vx, vy, wz = cmd.linear.x, cmd.linear.y, cmd.angular.z
        # 몸체: 미끄러진 속도
        body = Twist()
        body.linear.x = vx * self.sx * (1.0 + random.gauss(0.0, self.noise))
        body.linear.y = vy * self.sy * (1.0 + random.gauss(0.0, self.noise))
        body.angular.z = wz * self.sw * (1.0 + random.gauss(0.0, self.noise)) + self.yaw_per_lateral * vy
        self.body_pub.publish(body)
        # 바퀴 odom: 명령(바퀴가 돈 만큼)을 적분
        if self.last is not None:
            dt = (now - self.last).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                x, y, yaw = self.pose
                mid = yaw + 0.5 * wz * dt
                x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
                y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
                self.pose = [x, y, math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt))]
        self.last = now
        x, y, yaw = self.pose
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        odom.twist.twist = cmd
        self.odom_pub.publish(odom)
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = MecanumSlipSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
