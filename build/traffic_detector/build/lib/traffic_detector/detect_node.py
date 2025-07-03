import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # Pretrained YOLOv8
        self.get_logger().info('✅ YOLOv8 Detector Node Started')

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)
        annotated = results[0].plot()
        cv2.imwrite('/home/ponnu/ros2_ws/output.jpg', annotated)
        self.get_logger().info('📸 Saved: output.jpg')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
