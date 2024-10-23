import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#
import cv2 as cv
#
import onnxruntime
from torchvision import transforms
#
import json
import random
import numpy as np

# This class is used for the inforence of ONNX models
class OnnxInferenceNode(Node):
    def __init__(self, cameras_topic: list, cameras_ids: list,           # Cameras
                       model_name: str, mode: str,                       # Model and mode
                       model_type: str, json_file_path: str,             # Model type and class dictionary for YOLO
                       size_img = (640, 640), anns_dict = None,          # Image size and class dictionary for custom
                       confidence = 0.5, camera_id = 0):                 # Confidence threshold and camera id for real time
        super().__init__('onnx_inference')                               # Init the node
        # Cameras
        self.cameras_topic = cameras_topic                               # List of cameras topic
        self.cameras_ids = cameras_ids                                   # List of cameras ids
        self.camera_id = camera_id                                       # Camera id # Used for real time
        # Model and session
        self.session = self.init_session(model_name)                     # Init the session with the model
        # Create the transform
        # Needs to preprocess the image before the inference
        self.transform = transforms.Compose([transforms.ToPILImage(), 
                                             transforms.Resize(size_img), 
                                             transforms.ToTensor()])     # Transform the image
        # Load the class dictionary
        if model_type == 'yolo':
            self.anns_dict = self.load_class_dict(json_file_path)        # Load the class dictionary for YOLO
        else:
            self.anns_dict = anns_dict                                   # Load the class dictionary
        # Tools
        self.convert = CvBridge()                                        # Used for convert the ROS image into CV
        self.confidence = confidence                                     # Confidence threshold
        self.class_colors = self.generate_class_colors()                 # Generate the class colors
        self.pixel_format = 'bgr8'                                       # Set the pixel format

        # Frames
        self.frames = []                                                 # List of frames

        # Mode selection
        if mode == 'real_time':
            self.start_real_time()
        elif mode == 'inference':
            #self.inference_sender()
            pass
        else:
            print("Mode not supported")
            exit()

    def init_session(self, model_path):
        return onnxruntime.InferenceSession(model_path)
    
    # Infer the image in real time and show the result for the specific camera
    def start_real_time(self):
        # Create a subscription for the camera
        print(f'Setting the subscription for the camera {self.camera_id} on the topic {self.cameras_topic[self.camera_id]}')
        subscription = self.create_subscription(Image, self.cameras_topic[self.camera_id], self.infer_image_real_time, 10)

    def infer_image_real_time(self, msg):
        # Take the image from the camera topic
        print(f'Infering the image from the camera {self.camera_id}')
        print(f'Converting the image from the camera {self.camera_id}')
        frame = self.convert.imgmsg_to_cv2(msg, self.pixel_format)
        # Preprocess the image
        frame_tensor = self.transform(frame).unsqueeze(0)
        frame_numpy = frame_tensor.numpy()
        # Infer the image
        output = self.session.run(['output0'], {'images': frame_numpy}) # yolo11x use 'output0' as output and 'images' as input #TODO: Change this
        # Postprocess the image
        frame_ann = self.annotate_image(frame_numpy, output)
        print(f'Annotated the image from the camera {self.camera_id}')
        cv.imshow(f"YOLOv8 prediction from {self.camera_id}", frame_ann)
        cv.waitKey(1)

    def annotate_image(self, frame, output):
        # Annotate the image
        bounding_boxes = output[0]
        segmentations = output[1] if len(output) > 1 else None           # Assuming segmentations are in the second output if available

        # Draw bounding boxes
        for i in range(bounding_boxes.shape[1]):
            box = bounding_boxes[0, i, :]  # Unpack box values
            if len(box) >= 6:
                x1, y1, x2, y2, confidence, class_id = box[:6]
                if confidence > self.confidence:                         # Confidence threshold
                    label = f"{self.anns_dict.get(int(class_id), 'unknown')} ({confidence:.2f})"
                    color = self.class_colors.get(int(class_id), (0, 255, 0))
                    cv.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    cv.putText(frame, label, (int(x1), int(y1) - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        if segmentations is not None:
            # Draw segmentations
            for i in range(segmentations.shape[1]):
                mask = segmentations[0, i, :, :]
                class_id = i                                             # Assuming each mask corresponds to a class
                color = self.class_colors.get(int(class_id), (0, 255, 0))
                mask = mask > self.confidence
                frame[mask] = cv.addWeighted(frame[mask], 0.5, color, 0.5, 0)
        return frame

    # Generate a random color for each class
    def generate_class_colors(self):
        class_colors = {}
        for class_id in self.anns_dict.keys():
            class_colors[int(class_id)] = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        return class_colors
    
    # Take the class dictionary and save it in a json file
    def load_class_dict(self, json_file_path):
        with open(json_file_path, 'r') as file:
            data = json.load(file)
            class_dict = data.get("class", {})
        return class_dict