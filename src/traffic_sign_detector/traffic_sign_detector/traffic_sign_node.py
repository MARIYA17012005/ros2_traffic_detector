import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

        # Create output folder if not exists
        self.output_dir = os.path.expanduser('~/ros2_ws/output')
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info('Traffic Sign Detector Node Started')

    def image_callback(self, msg):
        self.get_logger().info("Received image for traffic sign detection")

        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize image for processing (optional)
            resized = cv2.resize(cv_image, (300, 300))

            # Save image to disk instead of showing it (WSL compatible)
            save_path = os.path.join(self.output_dir, "traffic_sign_detected.png")
            cv2.imwrite(save_path, resized)
            self.get_logger().info(f"Saved processed image to: {save_path}")

        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

