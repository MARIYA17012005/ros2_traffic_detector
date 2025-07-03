import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import glob

class DatasetPublisher(Node):
    def __init__(self):
        super().__init__('dataset_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.image_paths = sorted(glob.glob(os.path.expanduser(
            '~/ros2_ws/datasets/gtsrb/Final_Training/Images/00000/*.ppm')))
        self.index = 0
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(f"Loaded {len(self.image_paths)} images")

    def timer_callback(self):
        if self.index < len(self.image_paths):
            img_path = self.image_paths[self.index]
            frame = cv2.imread(img_path)
            if frame is not None:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published: {img_path}")
            self.index += 1
        else:
            self.get_logger().info("All images published.")
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = DatasetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
