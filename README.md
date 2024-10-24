# Faith-ROS

## Table of Contents
- [Installation](#installation)
    - [Installation ROS (humble)](#installation-humble)
- [Usage](#usage)
- [Nodes](#nodes)
  - [VideoPublisher](#videopublisher-node)
  - [YOLO Inference](#yolo-inference-node)
- [Build and Run](#build-and-run)
- [License](#license)

## Installation

### Requirements
- ROS2 Humble (Ubuntu 22.04)
- OpenCV (v4.5 or higher)
- Python3.10
- YOLOv8 (installed via `ultralytics` package)

## Installation-humble

Set locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
You will need to add the ROS 2 apt repository to your system

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ROS2 GPG key

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the repository to your sources list
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Then install it

```bash
sudo apt update && sudo apt upgrade
```

```bash
sudo apt install ros-humble-desktop
```

```bash
sudo apt install ros-humble-ros-base
```

```bash
sudo apt install ros-dev-tools
```
Then load the environment setup

```bash
source /opt/ros/humble/setup.bash
```

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
## Install setup
```bash
source install/setup.bash
```

## Camera node
This node captures video from a camera device and publishes the frames to a ROS2 topic

```bash
ros2 run faith_ros camera
```

## Inference node
For now run the inference script with python3:
```bash
ros2 run faith_ros inference <model_name>
```
