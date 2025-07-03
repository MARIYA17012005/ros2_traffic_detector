import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.model = YOLO('yolov8n.pt')  # Small YOLOv8 model
        self.get_logger().info('✅ YOLOv8 model loaded')

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)
        names = self.model.names
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label = names[cls_id]
            if label not in ['traffic light', 'stop sign']:
                continue
            self.get_logger().info(f"🟢 Detected: {label}")
        results[0].save(filename='/home/ponnu/ros2_ws/detected.jpg')

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    rclpy.spin(node)
    rclpy.shutdown()

