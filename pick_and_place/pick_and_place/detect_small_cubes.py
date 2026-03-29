#!/usr/bin/env python3

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class DetectSmallCubesNode(Node):
    def __init__(self):
        super().__init__('detect_small_cubes')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Transient local so late subscribers can still receive the detected small color.
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.small_color_pub = self.create_publisher(String, '/color_match/small_cube_color', latched_qos)

        self.min_area = int(self.declare_parameter('min_area', 120).value)
        self.report_every_n = int(self.declare_parameter('report_every_n', 10).value)

        self.frame_count = 0
        self.detected_once = False

        self.get_logger().info(
            'Small-cube detector started. Waiting to detect one small cube color on the table.'
        )

    def _build_masks(self, hsv):
        red1 = cv2.inRange(hsv, (0, 80, 50), (10, 255, 255))
        red2 = cv2.inRange(hsv, (170, 80, 50), (180, 255, 255))
        red = cv2.bitwise_or(red1, red2)

        green = cv2.inRange(hsv, (40, 80, 50), (85, 255, 255))
        blue = cv2.inRange(hsv, (95, 80, 50), (130, 255, 255))

        return {
            'red': red,
            'green': green,
            'blue': blue,
        }

    def image_callback(self, msg):
        if self.detected_once:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
            return

        self.frame_count += 1

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        masks = self._build_masks(hsv)

        best_color = None
        best_area = 0.0

        for color_name, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            area = 0.0
            for cnt in contours:
                area += cv2.contourArea(cnt)

            if area > best_area:
                best_area = area
                best_color = color_name

        if best_color is None or best_area < self.min_area:
            if self.frame_count % self.report_every_n == 0:
                self.get_logger().info('Waiting for a clear small-cube color detection...')
            return

        msg_out = String()
        msg_out.data = best_color
        self.small_color_pub.publish(msg_out)

        self.get_logger().info(
            f'Detected small cube color: {best_color} (area={best_area:.0f}). Published to /color_match/small_cube_color'
        )

        self.detected_once = True
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None

        self.get_logger().info('Small-cube detection complete. Exiting.')



def main(args=None):
    rclpy.init(args=args)
    node = DetectSmallCubesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
