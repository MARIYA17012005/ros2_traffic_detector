import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.bridge = CvBridge()

        self.image_paths = [
            os.path.expanduser('~/ros2_ws/src/traffic_sign_detector/test_images/sign.jpg'),
            os.path.expanduser('~/ros2_ws/src/traffic_sign_detector/test_images/light.jpg')
        ]
        self.current_index = 0

    def timer_callback(self):
        img_path = self.image_paths[self.current_index % len(self.image_paths)]
        self.current_index += 1
        image = cv2.imread(img_path)
        if image is None:
            self.get_logger().error(f"Failed to load image {img_path}")
            return

        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published image: {img_path}")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

