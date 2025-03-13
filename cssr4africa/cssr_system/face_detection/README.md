<div align="center">
<h1> Face and Mutual Gaze Detection </h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>


The **Face and Mutual Gaze Detection and Localization** package is a ROS package designed to detect multiple faces and evaluate their **mutual gaze** in real-time by subscribing to an image topic. It publishes an array of detected faces and their mutual gaze status to the **/faceDetection/data** topic. Each entry in the published data includes the **label ID** of the detected face, the **centroid** coordinates representing the center point of each face, and a boolean value indicating **mutual gaze** status as either **True** or **False**. 

## 📄 Documentation
The main documentation for this deliverable is found in [D4.2.2 Face and Mutual Gaze Detection and Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.2.pdf) that provides more details.

## 📦 Installation 
To set up the Face and Mutual Gaze Detection package on a Linux system, follow these steps:

1. Prerequisites
Make sure you are running a supported Linux distribution (e.g., Ubuntu 20.04 or later).
Install Python 3.10 and required tools.

2. Install Python 3.10 and Virtual Environment.
```sh
# Update system packages
sudo apt update && sudo apt upgrade -y

# Add the deadsnakes PPA for Python versions
sudo apt install software-properties-common -y
sudo add-apt-repository ppa:deadsnakes/ppa -y
sudo apt update

# Install Python 3.10
sudo apt install python3.10 python3.10-venv python3.10-distutils -y

# Verify Python installation
python3.10 --version
```
3. Set Up Virtual Environment
```sh
# Create a virtual environment:
cd $HOME
python3.10 -m venv faceDetection

# Activate the virtual environment:
source faceDetection/bin/activate

# Upgrade pip in the virtual environment:
pip install --upgrade pip
```

4. Install Required Packages
```sh
# Install PyTorch with CUDA support:
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install additional requirements:
pip install -r face_detection_requirements.txt
```

## Download the model
You need to download the [YOLO](https://drive.google.com/file/d/1-2n3ASmi7L0H6_1ssB_1yUHzuOKN9fTt/view?usp=sharing) and [Sixdrepnet360](https://drive.google.com/file/d/1u6JZ-jZNtd6DaNZybGOxvfr_mz8Hr4Cj/view?usp=sharing) models. Put them in the models folder of the face detetection package. 

## 🚀 Running the node
First, the configuration file must be setup with the right key-value pair before running the node. The configuration file is shown below. The configuration file can be in the package folder then the config folder. 

| Parameter                   | Description                                                | Range/Values               | Default Value |
|-----------------------------|------------------------------------------------------------|----------------------------|--------------|
| `algorithm`                 | Algorithm selected for face detection                      | `mediapipe` or `sixdrep`   | `sixdrep`    |
| `mp_facedet_confidence`     | Face detection confidence threshold (MediaPipe)            | 0.0 - 1.0                  | `0.5`        |
| `mp_headpose_angle`         | Head pose angle threshold (MediaPipe)                      | Degrees                    | `10`          |
| `centroid_max_distance`     | Maximum centroid distance for tracking                     | Positive integer           | `15`         |
| `centroid_max_disappeared`  | Maximum frames an object can disappear before deregistering| Positive integer           | `100`        |
| `sixdrepnet_confidence`     | Face detection confidence threshold (SixDrepNet)           | 0.0 - 1.0                  | `0.65`       |
| `sixdrepnet_headpose_angle` | Head pose angle threshold (SixDrepNet)                     | Degrees                    | `10`         |
| `sort_max_disappeared`      | Maximum number of frames an object can disappear for SORT  | Positive integer           | `5`          |
| `sort_min_hits`             | Minimum consecutive detections before confirming an object | Positive integer           | `3`          |
| `sort_iou_threshold`        | Intersection over Union (IoU) threshold for SORT tracking | 0.0 - 1.0                  | `0.3`        |
| `verbose_mode`              | Enable verbose logging                                     | `True` or `False`          | `True`       |

**Run the `faceDetection` from the `cssr_system` package:**

Source the workspace in first terminal:
  ```bash
  cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
  ```

Follow below steps, run in different terminals.\

  1️. Launch the robot and specifiy which camera to use. 
  ```bash
  roslaunch cssr_system cssrSystemLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface> camera:=<camera>
  ```

The default camera is set to the realsense.

  2️. Then run the Face and gaze detection and Localization.
  ```bash
  rosrun cssr_system face_detection_application.py
  ```

##  🖥️ Output
The node publishes the detected faces and their corresponding centroid, and the a boolean array whether a mutual gaze is established or not. When running in the verbose it display the OpenCv annotated color image and depth image that could help to visualize the result obtained. 

##  💡 Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:yohanneh@andrew.cmu.edu">yohanneh@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>


## 📜License
Copyright (C) 2023 CSSR4Africa Consortium  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme