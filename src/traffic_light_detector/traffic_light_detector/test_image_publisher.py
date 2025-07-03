import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(2.0, self.publish_image)

        # ✅ Use absolute paths to images
        package_dir = os.path.dirname(__file__)
        self.image_files = [
            os.path.join(package_dir, 'red-light.jpg'),
            os.path.join(package_dir, 'green-light.jpg'),
            os.path.join(package_dir, 'empty-road.jpg')
        ]
        self.index = 0
        self.get_logger().info("✅ Test Image Publisher started")

    def publish_image(self):
        img_path = self.image_files[self.index % len(self.image_files)]
        frame = cv2.imread(img_path)
        if frame is not None:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.get_logger().info(f"📷 Published: {os.path.basename(img_path)}")
        else:
            self.get_logger().error(f"❌ Failed to read image: {img_path}")
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
