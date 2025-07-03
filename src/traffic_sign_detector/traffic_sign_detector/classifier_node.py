import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import tensorflow as tf  # Optional for real models

class TrafficSignClassifier(Node):
    def __init__(self):
        super().__init__('traffic_sign_classifier')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info("Traffic Sign Classifier Node Started")

    def image_callback(self, msg):
        self.get_logger().info("Received image for classification")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Resize and normalize (dummy preprocess)
        img = cv2.resize(cv_image, (32, 32))
        img = img / 255.0
        img = np.expand_dims(img, axis=0)

        # Dummy classification output
        predicted_class = "STOP SIGN"  # Placeholder
        self.get_logger().info(f"Predicted: {predicted_class}")

        # Save the image with label overlay
        cv2.putText(cv_image, predicted_class, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        save_path = os.path.expanduser("~/ros2_ws/output/classified_sign.png")
        cv2.imwrite(save_path, cv_image)
        self.get_logger().info(f"Saved classified image to {save_path}")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

