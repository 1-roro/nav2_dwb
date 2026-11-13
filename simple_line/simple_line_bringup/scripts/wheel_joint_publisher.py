#!/usr/bin/env python3
"""
메카넘 바퀴 넷의 각도를 joint_states 로 낸다 (RViz 에서 바퀴가 돌고, robot_state_publisher 가 바퀴 TF 를 낸다).
시뮬레이션에만 쓴다. 실제 로봇은 모터 드라이버가 엔코더 값으로 joint_states 를 낸다.

odom 의 몸체 속도 (vx, vy, wz) 에서 메카넘 역기구학으로 바퀴 속도를 구해 적분한다 (X 배치, 반지름 r,
바퀴 축 사이 반 길이 lx, 반 폭 ly):
  ω_fl = (vx - vy - (lx + ly) wz) / r      ω_fr = (vx + vy + (lx + ly) wz) / r
  ω_rl = (vx + vy - (lx + ly) wz) / r      ω_rr = (vx - vy + (lx + ly) wz) / r
mecanum_slip:=True 이면 odom 은 바퀴 odom (명령을 적분) 이라, 바퀴는 명령대로 돌고 몸체는 덜 간다 (실제와 같다).
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

NAMES = ['wheel_front_left_joint', 'wheel_front_right_joint', 'wheel_rear_left_joint', 'wheel_rear_right_joint']


class WheelJointPublisher(Node):

    def __init__(self):
        super().__init__('wheel_joint_publisher')
        self.r = self.declare_parameter('wheel_radius', 0.04).value
        self.k = self.declare_parameter('half_length', 0.11).value + self.declare_parameter('half_width', 0.1325).value
        self.angles = [0.0] * 4
        self.twist = None
        self.last = None
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'twist', m.twist.twist), 10)
        self.create_timer(1.0 / 30.0, self.step)

    def step(self):
        now = self.get_clock().now()
        if self.last is not None and self.twist is not None:
            dt = (now - self.last).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                vx, vy, wz = self.twist.linear.x, self.twist.linear.y, self.twist.angular.z
                speeds = [vx - vy - self.k * wz, vx + vy + self.k * wz, vx + vy - self.k * wz, vx - vy + self.k * wz]
                self.angles = [a + s / self.r * dt for a, s in zip(self.angles, speeds)]
        self.last = now
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = NAMES
        msg.position = list(self.angles)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = WheelJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
