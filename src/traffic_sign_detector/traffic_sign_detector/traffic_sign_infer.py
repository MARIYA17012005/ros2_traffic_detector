from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import rclpy
import cv2

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')
        self.get_logger().info("📌 Starting TrafficSignDetector node...")

        model_path = '/home/ponnu/ros2_ws/src/traffic_sign_detector/models/best.pt'
        self.model = YOLO(model_path)
        self.get_logger().info("✅ Model loaded")

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info("📷 Image received")

        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO inference
        results = self.model(frame)
        boxes = results[0].boxes

        if boxes is not None and len(boxes) > 0:
            classes = boxes.cls.tolist()
            names = [self.model.names[int(cls)] for cls in classes]
            self.get_logger().info(f"🧠 Inference done. Detected: {names}")
        else:
            self.get_logger().info("🚫 No traffic signs detected.")


def main():
    rclpy.init()
    node = TrafficSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

