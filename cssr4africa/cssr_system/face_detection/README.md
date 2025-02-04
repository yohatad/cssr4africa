<div align="center">
<h1> Face and Mutual Gaze Detection </h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>


The **Face and Mutual Gaze Detection and Localization** package is a ROS package designed to detect multiple faces and evaluate their **mutual gaze** in real-time by subscribing to an image topic. It publishes an array of detected faces and their mutual gaze status to the **/faceDetection/data** topic. Each entry in the published data includes the **label ID** of the detected face, the **centroid** coordinates representing the center point of each face, and a boolean value indicating **mutual gaze** status as either **True** or **False**. 

## Documentation
The main documentation for this deliverable is found in [D4.2.1 Face and Mutual Gaze Detection and Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.1.pdf) that provides more details.

## Installation 
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
python3.10 -m venv face_gaze_detection

# Activate the virtual environment:
source face_gaze_detection/bin/activate

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

## Running the node
First, the configuration file must be setup with the right key-value pair before running the node. The configuration file is shown below. The configuration file can be in the package folder then the config folder. 

| Parameter                   | Description                                                | Range/Values               |
|-----------------------------|------------------------------------------------------------|----------------------------|
| `camera`                    | Type of camera used for input                              | `RealSense` or `Pepper`    |
| `algorithm`                 | Algorithm selected for face detection                      | `mediapipe` or `sixdrep`   |
| `centroid_max_distance`     | Maximum centroid distance for tracking                     | Positive integer           |
| `centroid_max_disappeared`  | Maximum frames an object can disappear before deregistering| Positive integer           |
| `mp_facedet_confidence`     | Face detection confidence threshold (MediaPipe)            | 0.0 - 1.0                  |
| `mp_headpose_angle`         | Head pose angle threshold (MediaPipe)                      | Degrees                    |
| `sixdrepnet_confidence`     | Face detection confidence threshold (SixDrepNet)           | 0.0 - 1.0                  |
| `sixdrepnet_headpose_angle` | Head pose angle threshold (SixDrepNet)                     | Degrees                    |
| `verboseMode`               | Enable verbose logging                                     | `True` or `False`          |

After setting up the right configuration, run the following command. 
1. Launch Pepper's camera or Intel real sense cameara. `Make sure the resoution of the depth and color camera is the same` 

2. Then run the Face and gaze detection and Localization.
    ```sh
    rosrun face_detection face_detection_application.py
    ```

## Output
The node publishes the detected faces and their corresponding centroid, and the a boolean array whether a mutual gaze is established or not. When running in the verbose it display the OpenCv annotated color image and depth image that could help to visualize the result obtained. 


## License

Copyright (C) 2023 CSSR4Africa Consortium  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme