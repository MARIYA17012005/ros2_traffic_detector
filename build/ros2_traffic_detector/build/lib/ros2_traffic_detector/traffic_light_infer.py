# traffic_light_infer.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # Load your trained YOLOv8 model
        model_path = os.path.expanduser('runs/detect/traffic_light_detector4/weights/best.pt')
        self.model = YOLO(model_path)

        # Assuming your model class index mapping is:
        # 0: red light, 1: green light, 2: yellow light
        self.class_names = ['Red', 'Green', 'Yellow']

        self.get_logger().info("Traffic light detection started")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO detection
        results = self.model(cv_image)

        detected_labels = set()

        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                if cls_id < len(self.class_names):
                    label = self.class_names[cls_id]
                    detected_labels.add(label)

        # Print detected lights (if any)
        if detected_labels:
            self.get_logger().info(f"Detected traffic lights: {', '.join(detected_labels)}")
        else:
            self.get_logger().info("No traffic light detected")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

