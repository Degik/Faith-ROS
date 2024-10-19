# Faith-ROS

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Nodes](#nodes)
  - [VideoPublisher](#videopublisher-node)
  - [YOLO Inference](#yolo-inference-node)
- [Build and Run](#build-and-run)
- [License](#license)

## Installation

### Requirements
- ROS2 Humble
- OpenCV (v4.5 or higher)
- Cv_bridge
- Python3.10
- YOLOv8 (installed via `ultralytics` package)

### Installing Dependencies

To install the required ROS2 and OpenCV libraries:

```bash
sudo apt update
sudo apt install ros-humble-desktop python3-opencv ros-humble-cv-bridge
```

To install YOLOv8 for inference

```bash
pip install ultralytics
```

After cloning the repository, navigate to the workspace and build the package using colcon

```bash
colcon build
```

## Camera node
This node captures video from a camera device and publishes the frames to a ROS2 topic

```bash
ros2 run ros_test1 camera
```

## Inference node
For now run the inference script with python3:
```bash
python3 src/inference.py
```
