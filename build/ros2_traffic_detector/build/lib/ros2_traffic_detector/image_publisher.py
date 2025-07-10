import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.get_logger().info("📸 ImagePublisher node started")

    def timer_callback(self):
        img_path = '/home/ponnu/ros2_ws/sample.jpg'
        img = cv2.imread(img_path)
        if img is None:
            self.get_logger().error(f"❌ Could not load image from {img_path}")
            return

        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info(f"✅ Published image to /camera/image_raw")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
