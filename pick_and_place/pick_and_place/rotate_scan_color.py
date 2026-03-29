#!/usr/bin/env python3

import math
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class RotateScanColorNode(Node):
    def __init__(self):
        super().__init__('rotate_scan_color')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        target_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.target_pub = self.create_publisher(String, '/color_match/target_big_cube', target_qos)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 20)

        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.picked_color_sub = self.create_subscription(
            String,
            '/color_match/picked_cube_color',
            self._picked_color_cb,
            latched_qos,
        )
        self.start_scan_sub = self.create_subscription(
            Bool,
            '/color_match/start_scan',
            self._start_scan_cb,
            latched_qos,
        )

        self.bridge = CvBridge()

        self.angular_speed = float(self.declare_parameter('angular_speed', 0.28).value)
        self.min_angular_speed = float(self.declare_parameter('min_angular_speed', 0.08).value)
        self.yaw_kp = float(self.declare_parameter('yaw_kp', 1.2).value)
        self.rotate_timeout_sec = float(self.declare_parameter('rotate_timeout_sec', 15.0).value)
        self.fine_correct_timeout_sec = float(self.declare_parameter('fine_correct_timeout_sec', 1.5).value)
        self.target_deg = float(self.declare_parameter('target_deg', 90.0).value)
        self.yaw_tolerance = float(self.declare_parameter('yaw_tolerance', 0.03).value)
        self.min_area_big = int(self.declare_parameter('min_area_big', 700).value)
        self.odom_wait_timeout_sec = float(self.declare_parameter('odom_wait_timeout_sec', 8.0).value)
        # 0 or negative means wait indefinitely for /color_match/picked_cube_color
        self.picked_color_wait_timeout_sec = float(
            self.declare_parameter('picked_color_wait_timeout_sec', 0.0).value
        )
        self.use_open_loop_if_no_odom = bool(
            self.declare_parameter('use_open_loop_if_no_odom', True).value
        )
        # 0 or negative means unlimited scan rotations until a match is found.
        self.max_scan_steps = int(self.declare_parameter('max_scan_steps', 0).value)

        self.current_yaw = None
        self.current_image = None
        self.picked_color = None
        self.scan_start_requested = False

        self.rotation_labels = ['left', 'behind', 'right']

        self.get_logger().info(
            'Rotate-scan matcher started. Waiting for /color_match/start_scan and /color_match/picked_cube_color'
        )

    def _picked_color_cb(self, msg):
        self.picked_color = msg.data.strip().lower()
        self.get_logger().info(f'Received picked cube color: {self.picked_color}')

    def _start_scan_cb(self, msg):
        self.scan_start_requested = bool(msg.data)
        if self.scan_start_requested:
            self.get_logger().info('Received scan start trigger on /color_match/start_scan')

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _image_cb(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')

    def _stop_base(self):
        self.cmd_pub.publish(Twist())

    def _rotate_left_90(self):
        if self.current_yaw is None:
            if not self.use_open_loop_if_no_odom:
                return False

            # Fallback: rotate by time when odometry is not available.
            target_rad = math.radians(self.target_deg)
            omega = max(abs(self.angular_speed), 0.05)
            rotate_time = target_rad / omega
            t0 = time.time()

            while rclpy.ok() and (time.time() - t0) < rotate_time:
                cmd = Twist()
                cmd.angular.z = abs(self.angular_speed)
                self.cmd_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.03)

            self._stop_base()
            time.sleep(0.25)
            return True

        start_yaw = self.current_yaw
        target_rad = math.radians(self.target_deg)
        target_yaw = normalize_angle(start_yaw + target_rad)
        timeout = self.rotate_timeout_sec
        t0 = time.time()

        while rclpy.ok() and (time.time() - t0) < timeout:
            err = normalize_angle(target_yaw - self.current_yaw)
            if abs(err) <= self.yaw_tolerance:
                break

            # Closed-loop turn command: fast when far, slow when near target.
            omega = self.yaw_kp * err
            if abs(omega) < self.min_angular_speed:
                omega = math.copysign(self.min_angular_speed, err)
            omega = max(-abs(self.angular_speed), min(abs(self.angular_speed), omega))

            cmd = Twist()
            cmd.angular.z = omega
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.02)

        # Fine correction phase to reduce residual heading error.
        t1 = time.time()
        while rclpy.ok() and (time.time() - t1) < self.fine_correct_timeout_sec:
            err = normalize_angle(target_yaw - self.current_yaw)
            if abs(err) <= self.yaw_tolerance:
                break

            omega = max(-0.15, min(0.15, 1.0 * err))
            if abs(omega) < 0.05:
                omega = math.copysign(0.05, err)

            cmd = Twist()
            cmd.angular.z = omega
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.02)

        self._stop_base()
        time.sleep(0.15)
        return True

    def _build_masks(self, hsv):
        red1 = cv2.inRange(hsv, (0, 80, 50), (10, 255, 255))
        red2 = cv2.inRange(hsv, (170, 80, 50), (180, 255, 255))
        red = cv2.bitwise_or(red1, red2)
        green = cv2.inRange(hsv, (40, 80, 50), (85, 255, 255))
        blue = cv2.inRange(hsv, (95, 80, 50), (130, 255, 255))
        return {'red': red, 'green': green, 'blue': blue}

    def _detect_big_cube_color(self, timeout_sec=3.0):
        t0 = time.time()
        best_color = None
        best_area = 0.0

        while rclpy.ok() and (time.time() - t0) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.current_image is None:
                continue

            image = self.current_image
            h, w = image.shape[:2]
            y1, y2 = int(0.25 * h), int(0.9 * h)
            x1, x2 = int(0.2 * w), int(0.8 * w)
            roi = image[y1:y2, x1:x2]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            masks = self._build_masks(hsv)

            for color_name, mask in masks.items():
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                area = 0.0
                for cnt in contours:
                    area += cv2.contourArea(cnt)
                if area > best_area:
                    best_area = area
                    best_color = color_name

        if best_color is None or best_area < self.min_area_big:
            return None

        self.get_logger().info(f'Detected big cube color: {best_color} (area={best_area:.0f})')
        return best_color

    def run(self):
        self.get_logger().info('Waiting for scan trigger and picked cube color...')
        start_wait = time.time()
        last_log = start_wait
        while rclpy.ok() and (not self.scan_start_requested or not self.picked_color):
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()
            if now - last_log > 5.0:
                self.get_logger().info('Still waiting for /color_match/start_scan and /color_match/picked_cube_color ...')
                last_log = now
            if self.picked_color_wait_timeout_sec > 0.0 and (now - start_wait) > self.picked_color_wait_timeout_sec:
                self.get_logger().error('Timed out waiting for scan trigger and picked color.')
                return False

        # Odometry is preferred, but optional. If unavailable, open-loop rotation is used.
        if self.current_yaw is None and self.odom_wait_timeout_sec > 0.0:
            self.get_logger().info('Picked color received. Waiting briefly for odometry...')
            start_wait = time.time()
            while rclpy.ok() and self.current_yaw is None and (time.time() - start_wait) < self.odom_wait_timeout_sec:
                rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_yaw is None:
            if self.use_open_loop_if_no_odom:
                self.get_logger().warn('No odometry available. Using open-loop timed rotations.')
            else:
                self.get_logger().error('No odometry available for rotate-scan.')
                return False

        step_count = 0
        while rclpy.ok():
            if self.max_scan_steps > 0 and step_count >= self.max_scan_steps:
                self.get_logger().error(
                    f'No matching big cube found after {step_count} scan rotations.'
                )
                return False

            label = self.rotation_labels[step_count % len(self.rotation_labels)]
            self.get_logger().info(f'Rotating left to check {label} big cube...')
            if not self._rotate_left_90():
                self.get_logger().error('Failed to rotate 90 degrees.')
                return False

            detected = self._detect_big_cube_color(timeout_sec=3.0)
            if detected is None:
                self.get_logger().warn(f'No clear big-cube color detected at {label}.')
                step_count += 1
                continue

            if detected == self.picked_color:
                out = String()
                out.data = label
                self.target_pub.publish(out)
                self.get_logger().info(
                    f'Match found: small={self.picked_color}, big={detected}, target={label}. Published /color_match/target_big_cube'
                )
                return True

            self.get_logger().info(
                f'No match at {label}: small={self.picked_color}, big={detected}. Rotating again...'
            )
            step_count += 1



def main(args=None):
    rclpy.init(args=args)
    node = RotateScanColorNode()
    success = False
    try:
        success = node.run()
    finally:
        node._stop_base()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    main()
