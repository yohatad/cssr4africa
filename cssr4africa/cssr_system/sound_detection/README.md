# Sound Detection and Localization

<img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">

The **Sound Detection and Localization package** is a ROS package designed to detect conspicuous sounds, specifically human voices, and determine their direction of arrival in real-time by processing audio signals from the robot's microphones. The package calculates the azimuth angle of arrival, representing the sound's direction on the horizontal plane relative to the robot's Cartesian head frame. It publishes this angle to the **/soundDetection/direction** topic, allowing the robot to direct its gaze toward the detected sound source. Additionally, the captured audio signal is published to the **/soundDetection/signal** topic, from onset to offset of the detected sound. In verbose mode, the module provides diagnostic output to the terminal for enhanced debugging. This package enables the robot to localize and respond to human voices, filtering out ambient noise and reverberation to maintain precise and responsive auditory localization in real-time

## Documentation
The main documentation for this deliverable is found in [D4.3.1 Sound detection and Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.3.1.pdf) that provides more details.

## Installation 
To install the sound detection and localization package, install the following packages.
First create a virtual environment and install the required packages. You need to configure the right GPU driver to use torch with CUDA support.

```sh
cd $HOME
python3 -m venv sound_detection
source sound_detection/bin/activate
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install -r face_detection_requirements.txt
```

## Download the model
You need to download the [NSNet](https://drive.google.com/file/d/1G8OPmx8RlrEbagMmK0-OG023XqB45ta_/view?usp=sharing). Put them in the models folder of the sound detection package. 

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
| `deepsort_max_age`          | Maximum age for DeepSort tracker                           | Positive integer           |
| `deepsort_max_iou_distance` | Maximum IoU distance for DeepSort tracker                  | 0.0 - 1.0                  |
| `deepsort_n_init`           | Consecutive detections to confirm a track (DeepSort)       | Positive integer           |
| `verboseMode`               | Enable verbose logging                                     | `True` or `False`          |

After setting up the right configuration, run the following command. 
1. Launch Pepper's camera or Intel real sense cameara. `Make sure the resoution of the depth and color camera is the same` 

2. Then run the Face and gaze detection and Localization.
    ```sh
    rosrun face_detection faceDetectionApplication.py
    ```

## Output
The node publishes the detected faces and their corresponding centroid, and the a boolean array whether a mutual gaze is established or not. When running in the verbose it display the OpenCv annotated color image and depth image that could help to visualize the result obtained. 


## License

Copyright (C) 2023 CSSR4Africa Consortium  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme