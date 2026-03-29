#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelMotionTest(Node):
    def __init__(self):
        super().__init__('cmd_vel_motion_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_for_duration(self, linear_x: float, angular_z: float, duration_s: float, rate_hz: float = 20.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)

        period = 1.0 / rate_hz
        end_time = time.time() + duration_s

        while rclpy.ok() and time.time() < end_time:
            self.pub.publish(msg)
            time.sleep(period)

    def stop(self):
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMotionTest()

    try:
        node.get_logger().info('Step 1: forward 3s')
        node.send_for_duration(linear_x=0.25, angular_z=0.0, duration_s=3.0)

        node.get_logger().info('Step 2: stop 1s')
        node.send_for_duration(linear_x=0.0, angular_z=0.0, duration_s=1.0)

        node.get_logger().info('Step 3: rotate left 2s')
        node.send_for_duration(linear_x=0.0, angular_z=0.5, duration_s=2.0)

        node.get_logger().info('Step 4: stop 1s')
        node.send_for_duration(linear_x=0.0, angular_z=0.0, duration_s=1.0)

        node.get_logger().info('Step 5: forward 3s')
        node.send_for_duration(linear_x=0.25, angular_z=0.0, duration_s=3.0)

        node.stop()
        node.get_logger().info('Motion test complete')
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
