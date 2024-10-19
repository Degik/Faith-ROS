# ROS2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# OpenCV
import cv2 as cv
# Yolo
from ultralytics import YOLO

class YOLOInferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')
        self.subscription = self.create_subscription(Image, 'camera', self.infer_image, 10)
        self.convert = CvBridge()           # Used for convert the ROS image into CV
        self.model = YOLO('yolov8m.pt')     # Load the model yolov8m
    
    def infer_image(self, msg):
        frame = self.convert.imgmsg_to_cv2(msg, 'bgr8') # Convert the ROS image into CV
        results = self.model(frame)                     # Inference
        
        # https://docs.ultralytics.com/reference/engine/results/#ultralytics.engine.results.Results.numpy
        annotated_frame = results[0].plot()                # Annotate the frame
        cv.imshow("YOLOv8 prediction", annotated_frame)
        cv.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    node = YOLOInferenceNode()
    rclpy.spin(node)
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()