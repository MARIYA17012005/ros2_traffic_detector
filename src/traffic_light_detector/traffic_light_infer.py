import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.model = YOLO('best.pt')  # ensure 'best.pt' is in the same folder
        self.get_logger().info("✅ YOLOv8 model loaded for traffic light detection")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model.predict(source=cv_image, conf=0.3, imgsz=640, verbose=False)

        for r in results:
            boxes = r.boxes
            for box in boxes:
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                conf = box.conf[0].item()
                cls = int(box.cls[0].item())
                label = f"TL {conf:.2f}"
                cv2.rectangle(cv_image, tuple(xyxy[:2]), tuple(xyxy[2:]), (0,255,0), 2)
                cv2.putText(cv_image, label, tuple(xyxy[:2]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # Save the image result
        cv2.imwrite('/home/ponnu/output_detected.jpg', cv_image)
        self.get_logger().info("🖼 Detection complete - image saved.")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
