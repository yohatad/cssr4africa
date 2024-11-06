# Face and Mutual Gaze Detection

<img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">

The **Face and Mutual Gaze Detection and Localization** package is a ROS package designed to detect multiple faces and evaluate their **mutual gaze** in real-time by subscribing to an image topic. It publishes an array of detected faces and their mutual gaze status to the **/faceDetection/data** topic. Each entry in the published data includes the **label ID** of the detected face, the **centroid** coordinates representing the center point of each face, and a boolean value indicating **mutual gaze** status as either **True** or **False**. 

## Documentation
The main documentation for this deliverable is found in [D5.5.1 Face and Mutual Gaze Detection and Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.1.pdf) that provides more details.

## Installation 
To install the face and mutual gaze detection package, install the following packages.
First create a virtual environment and install the required packages. You need to configure the right GPU driver to use torch with CUDA support. 

```sh
cd $HOME
python3 -m venv face_gaze_detection
source face_gaze_detection/bin/activate
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install -r face_detection_requirements.txt
```

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

```sh
rosrun face_detection faceDetectionApplication.py
```

## Output
The node publishes the detected faces and their corresponding centroid, and the a boolean array whether a mutual gaze is established or not. When running in the verbose it display the OpenCv Image and Depth Image that could help to visualize the result obtained. 