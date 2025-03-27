<div align="center">
<h1> Person Detection and Localization </h1>
</div>

<div align="center">
  <img src="../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The **Person Detection and Localization** package is a ROS package designed to detect multiple persons in real-time by subscribing to color and depth image topics. It publishes an array of detected persons to the **/personDetection/data** topic. Each entry in the published data includes the **label ID** of the detected person, the **centroid** coordinates representing the center point of each person, the **width** and **height** of the bounding box, and the **depth** information in meters.

# 📄 Documentation
The main documentation for this deliverable provides more details about the implementation and usage of the person detection system.

# 🛠️ Installation 

Install the required software components to instantiate and set up the development environment for controlling the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). This includes downloading the model files and putting them in the models directory. 

To set up the Person Detection package on a Linux system, follow these steps:

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
python3.8 -m venv ~/workspace/pepper_rob_ws/person_detection

# Activate the virtual environment:
source ~/workspace/pepper_rob_ws/person_detection/bin/activate

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
pip install -r person_detection_requirements.txt
```


# 🔧 Configuration Parameters
The following table provides the key-value pairs used in the configuration file:

| Parameter                   | Description                                                      | Range/Values            | Default Value |
|-----------------------------|------------------------------------------------------------------|-------------------------|---------------|
| `use_compressed`            | Use compressed ROS image topics                                  | `True`, `False`         | `False`       |
| `confidence_iou_threshold`  | Confidence threshold for person detection                       | `[0.0 - 1.0]`           | `0.5`         |
| `sort_max_disappeared`      | Maximum frames allowed for disappearance in SORT tracking        | Positive integer        | `50`          |
| `sort_max_hits`             | Minimum consecutive detections to confirm object tracking (SORT) | Positive integer        | `3`           |
| `sort_iou_threshold`        | IoU threshold for SORT tracker                                   | `[0.0 - 1.0]`           | `0.5`         |
| `verbose_mode`              | Enable visualization using OpenCV windows and detailed logging   | `True`, `False`         | `False`       |

> **Note:**  
> Enabling **`verbose_mode`** (`True`) will activate real-time visualization via OpenCV windows. 

# 🚀 Running the node
**Run the `personDetection` from the `cssr_system` package:**

Source the workspace in first terminal:
  ```bash
  cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
  ```

Follow below steps, run in different terminals.

  1️. Launch the robot and specify which camera to use. 
  ```bash
  roslaunch cssr_system person_detection_robot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface> camera:=<camera>
  ```

  The default camera is set to the realsense.

  2️. Then run the Person Detection and Localization.

  In a new terminal activate the python environment. 
  ```bash
  source ~/workspace/pepper_rob_ws/person_detection/bin/activate
  ```

  ```bash
  # Run the person_detection node
  rosrun cssr_system person_detection_application.py
  ```

#  🖥️ Output
The node publishes the detected persons and their corresponding centroid, the width and height of the bounding box, and the depth information in meters. When running in verbose mode, it displays the OpenCV annotated color image and depth image that helps visualize the results obtained.

Subscription to the topic **personDetection/data** allows verification of its publication status using the following command:

```bash
rostopic echo /personDetection/data
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

2025-03-26