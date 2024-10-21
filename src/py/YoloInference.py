import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# OpenCV
import cv2 as cv
# Yolo
from ultralytics import YOLO

# YOLOInferencenode class elaborates the inference of the YOLO model for the cameras and publish the results
class YOLOInferenceNode(Node):
    def __init__(self, cameras_topic: list, cameras_ids: list, model_name: str, mode: str):
        super().__init__('yolo_inference_node')
        # Cameras
        self.cameras_topic = cameras_topic              # List of cameras topic
        self.cameras_ids = cameras_ids                  # List of cameras ids

        # Tools
        self.convert = CvBridge()                       # Used for convert the ROS image into CV
        self.model = YOLO(model_name)                   # Load the model yolov8m
        self.pixel_format = 'bgr8'                      # Set the pixel format

        # Frames
        self.frames = []                                # List of frames

        # Mode selection
        if mode == 'real_time':
            self.infer_image_real_time(cameras_ids)
        elif mode == 'inference':
            self.inference_sender(cameras_topic)
        else:
            print("Mode not supported")
            exit()

    def inference_sender(self):
        subsbcriptions = []
        # Create a subscription for each camera topic
        for topic in self.cameras_topic:
            # Append the subscription to the list
            subsbcriptions.append(self.create_subscription(Image, topic, self.infer_image, 10))
        # Wait for the subscription to be ready
        self.send_frames()

    # Infer the image and show the result
    def infer_image(self, msg):
        # Infer the image
        frame = self.convert.imgmsg_to_cv2(msg, self.pixel_format)
        results = self.model(frame)
        annotated_frame = results[0].plot()
        self.frames.append(annotated_frame)

    # Infer the image in real time and show the result for the specific camera
    def infer_image_real_time(self, single_camera_id):
        # Create a subscription for the camera
        subscription = self.create_subscription(Image, self.cameras_topic[single_camera_id], self.infer_image, 10)
        # Wait for the subscription to be ready
        # https://docs.ultralytics.com/reference/engine/results/#ultralytics.engine.results.Results.numpy
        cv.imshow("YOLOv8 prediction", self.frames[0])
        cv.waitKey(1)


    # This method take the frames and topics elaborated and send them to master
    def send_frames(self, topics):
        # Spin once
        rclpy.spin_once(self)
        for i, frame in enumerate(self.frames):
            # Convert the frame into a ROS image
            msg = self.convert.cv2_to_imgmsg(frame, self.pixel_format)
            # Publish the image on the topic of the camera
            self.publisher = self.create_publisher(Image, topics[i], 10)
            self.publisher.publish(msg)
            # Remove the frame from the list
            self.frames.pop(i)