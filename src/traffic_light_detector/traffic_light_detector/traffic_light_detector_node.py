import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector_node')
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info("🚦 Traffic Light Detector Node started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color ranges for red and green
        red_lower = np.array([0, 100, 100])
        red_upper = np.array([10, 255, 255])
        green_lower = np.array([45, 100, 100])
        green_upper = np.array([75, 255, 255])

        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        if cv2.countNonZero(red_mask) > 500:
            self.get_logger().info("🔴 Red Light Detected")
        elif cv2.countNonZero(green_mask) > 500:
            self.get_logger().info("🟢 Green Light Detected")
        else:
            self.get_logger().info("⚪ No traffic light detected")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
