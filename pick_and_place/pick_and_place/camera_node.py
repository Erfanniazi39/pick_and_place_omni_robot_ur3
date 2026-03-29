import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Subscribe to camera images
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        self.current_image = None
        self.frame_count = 0
        self.get_logger().info('Camera node started. Listening for images from /camera/image_raw')

    def image_callback(self, msg):
        """Callback to get camera images"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Get image dimensions
            height, width = cv_image.shape[:2]
            self.frame_count += 1
            self.get_logger().info(f'Frame {self.frame_count}: {width}x{height}')
            
            # Store current image for processing
            self.current_image = cv_image
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        camera_node.get_logger().info('Node interrupted')
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
