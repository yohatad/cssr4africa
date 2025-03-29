<div align="center">
<h1> Face and Mutual Gaze Detection and Localization </h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The **Face and Mutual Gaze Detection and Localization** package is a ROS package designed to detect multiple faces and evaluate their **mutual gaze** in real-time by subscribing to an image topic. It publishes an array of detected faces and their mutual gaze status to the **/faceDetection/data** topic. Each entry in the published data includes the **label ID** of the detected face, the **centroid** coordinates representing the center point of each face, and a boolean value indicating **mutual gaze** status as either **True** or **False**, the **widht** and **height** of the bounding box.

# 📄 Documentation
The main documentation for this deliverable is found in [D4.2.2 Face and Mutual Gaze Detection and Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.2.pdf) that provides more details.

# 🛠️ Installation 

Install the required software components to instantiate and set up the development environment for controlling the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). This includes downloading the models files and putting in the models files directory. 

To set up the Face and Mutual Gaze Detection package on a Linux system, follow these steps:

1. Prerequisites  
Make sure you are running a supported Linux distribution (e.g., Ubuntu 20.04 or later).


2. Install Python 3.8 and Virtual Environment (If it isn't installed).
```sh
# To check if it is installed run the command below
python3.8 --version 

# If doesn't print the version number then it is not installed. Follow the steps below.
# Update system packages
sudo apt update && sudo apt upgrade -y

# Add the deadsnakes PPA for Python versions
sudo apt install software-properties-common -y
sudo add-apt-repository ppa:deadsnakes/ppa -y
sudo apt update

# Install Python 3.8
sudo apt install python3.8 python3.8-venv python3.8-distutils -y

# Verify Python installation
python3.8 --version
```
3. Set Up Virtual Environment
```sh
# Create a virtual environment:
cd $HOME
python3.8 -m venv ~/workspace/pepper_rob_ws/face_detection

# Activate the virtual environment:
source ~/workspace/pepper_rob_ws/face_detection/bin/activate

# Upgrade pip in the virtual environment:
pip install --upgrade pip
```

4. Install Required Packages
```sh
# Install onnxruntime package (Downloading pip wheel)
$ wget https://nvidia.box.com/shared/static/iizg3ggrtdkqawkmebbfixo7sce6j365.whl -O onnxruntime_gpu-1.16.0-cp38-cp38-linux_aarch64.whl

# Install pip wheel
$ pip3 install onnxruntime_gpu-1.16.0-cp38-cp38-linux_aarch64.whl
```
```sh
# Install additional requirements:
pip install -r face_detection_requirements.txt
```


# 🔧 Configuration Parameters
The following table provides the key-value pairs used in the configuration file:

| Parameter                   | Description                                                      | Range/Values            | Default Value |
|-----------------------------|------------------------------------------------------------------|-------------------------|---------------|
| `algorithm`                 | Algorithm selected for face detection                            | `mediapipe`, `sixdrep`  | `sixdrep`     |
| `use_compressed`            | Use compressed ROS image topics                                  | `True`, `False`         | `True`        |
| `mp_confidence`             | Face detection confidence threshold (MediaPipe)                  | `[0.0 - 1.0]`           | `0.5`         |
| `mp_headpose_angle`         | Head pose angle threshold in degrees (MediaPipe)                 | Positive integer        | `8`           |
| `centroid_max_distance`     | Maximum centroid distance for centroid tracking                  | Positive integer        | `15`          |
| `centroid_max_disappeared`  | Frames allowed before centroid tracker deregisters an object     | Positive integer        | `100`         |
| `sixdrepnet_confidence`     | Confidence threshold for face detection (SixDRepNet)             | `[0.0 - 1.0]`           | `0.65`        |
| `sixdrepnet_headpose_angle` | Head pose angle threshold in degrees (SixDRepNet)                | Positive integer        | `10`          |
| `sort_max_disappeared`      | Maximum frames allowed for disappearance in SORT tracking        | Positive integer        | `30`          |
| `sort_min_hits`             | Minimum consecutive detections to confirm object tracking (SORT) | Positive integer        | `20`          |
| `sort_iou_threshold`        | IoU threshold for SORT tracker                                   | `[0.0 - 1.0]`           | `0.3`         |
| `verbose_mode`              | Enable visualization using OpenCV windows and detailed logging   | `True`, `False`         | `True`        |

> **Note:**  
> Enabling **`verbose_mode`** (`True`) will activate real-time visualization via OpenCV windows. 

# 🚀 Running the node
**Run the `faceDetection` from the `cssr_system` package:**

Source the workspace in first terminal:
  ```bash
  cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
  ```

Follow below steps, run in different terminals.

  1️. Launch the robot and specifiy which camera to use. 
  ```bash
  roslaunch cssr_system face_detection_launch_robot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface> camera:=<camera>
  ```

  The default camera is set to the realsense.

  2️. Then run the Face and gaze detection and   Localization.

  In a new terminal activate the python environment. 
  ```bash
  source ~/workspace/pepper_rob_ws/face_person_detection/bin/activate
  ```

  ```bash
  # Run the face_detection node
  rosrun cssr_system face_detection_application.py
  ```

#  🖥️ Output
The node publishes the detected faces and their corresponding centroid, the width and height of the bounding box and the a boolean array whether a mutual gaze is established or not. When running in the verbose it display the OpenCv annotated color image and depth image that could help to visualize the result obtained. 

Subscription to the topic **faceDetection/data** allows verification of its publication status using the following command:

```bash
rostopic echo /faceDetection/data
```

# 💡 Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:yohanneh@andrew.cmu.edu">yohanneh@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>


# 📜License
Copyright (C) 2023 CSSR4Africa Consortium  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

2025-03-15