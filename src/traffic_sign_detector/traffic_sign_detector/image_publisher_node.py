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
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.bridge = CvBridge()

        # Load test image
        self.img_path = os.path.expanduser('~/ros2_ws/src/traffic_sign_detector/test_sign.png')
        self.cv_image = cv2.imread(self.img_path)

        if self.cv_image is None:
            self.get_logger().error(f"Could not load image from {self.img_path}")
        else:
            self.get_logger().info(f"Loaded test image: {self.img_path}")

    def timer_callback(self):
        if self.cv_image is not None:
            msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('Published image to /camera/image_raw')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

