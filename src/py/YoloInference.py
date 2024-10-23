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
        self.camera_id = 0                              # Camera id

        # Tools
        self.convert = CvBridge()                       # Used for convert the ROS image into CV
        self.model = YOLO(model_name)                   # Load the model yolov8m
        self.pixel_format = 'bgr8'                      # Set the pixel format

        # Frames
        self.frames = []                                # List of frames

        # Mode selection
        if mode == 'real_time':
            self.infer_image_real_time()
        elif mode == 'inference':
            self.inference_sender()
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
        rclpy.spin_once(self)
        self.send_frames()

    # Infer the image and show the result
    def infer_image_for_real_time(self, msg):
        # Infer the image
        print(f'Infering the image from the camera {self.camera_id}')
        print(f'Converting the image from the camera {self.camera_id}')
        frame = self.convert.imgmsg_to_cv2(msg, self.pixel_format)
        # https://docs.ultralytics.com/reference/engine/results/#ultralytics.engine.results.Results.numpy
        results = self.model(frame)
        annotated_frame = results[0].plot()
        print(f'Annotated the image from the camera {self.camera_id}')
        cv.imshow(f"YOLOv8 prediction from {self.camera_id}", annotated_frame)
        cv.waitKey(1)
        #self.frames.append(annotated_frame)

    def infer_image(self, msg):
        # Infer the image
        print(f'Infering the image from the camera {self.camera_id}')
        print(f'Converting the image from the camera {self.camera_id}')
        frame = self.convert.imgmsg_to_cv2(msg, self.pixel_format)
        # https://docs.ultralytics.com/reference/engine/results/#ultralytics.engine.results.Results.numpy
        results = self.model(frame)
        annotated_frame = results[0].plot()
        self.frames.append(annotated_frame)

    # Infer the image in real time and show the result for the specific camera
    def infer_image_real_time(self):
        # Create a subscription for the camera
        print(f'Setting the subscription for the camera {self.camera_id} on the topic {self.cameras_topic[self.camera_id]}')
        subscription = self.create_subscription(Image, self.cameras_topic[self.camera_id], self.infer_image_for_real_time, 10)

    # This method take the frames and topics elaborated and send them to master
    def send_frames(self):
        for i, frame in enumerate(self.frames):
            # Convert the frame into a ROS image
            msg = self.convert.cv2_to_imgmsg(frame, self.pixel_format)
            # Publish the image on the topic of the camera
            print(f'inf_{self.cameras_topic[i]}')
            self.publisher = self.create_publisher(Image, f"inf_{self.cameras_topic[i]}", 10) # ??? forse non si mette
            self.publisher.publish(msg)
            # Remove the frame from the list
            self.frames.pop(i)

    def set_camera(self, camera_id):
        self.camera_id = camera_id