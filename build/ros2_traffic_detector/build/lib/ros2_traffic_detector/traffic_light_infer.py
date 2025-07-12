# traffic_light_infer.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(String, '/traffic_light_status', 10)
        self.bridge = CvBridge()

        # Load your YOLO model
        model_path = '/home/ponnu/ros2_ws/src/traffic_light_detector/best.pt'
        self.model = YOLO(model_path)

        self.class_names = ['Red', 'Green', 'Yellow']  # class indices must match model training
        self.get_logger().info("Traffic light detection started")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image)

        detected_labels = set()

        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                if cls_id < len(self.class_names):
                    label = self.class_names[cls_id]
                    detected_labels.add(label)

        if detected_labels:
            light_status = ','.join(detected_labels)
            self.get_logger().info(f"Detected traffic lights: {light_status}")
        else:
            light_status = 'None'
            self.get_logger().info("No traffic light detected")

        # Publish the result
        msg = String()
        msg.data = light_status
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

