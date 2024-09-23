# Face and Mutual Gaze Detection

<img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:100%; height:auto;">

The face and mutual gaze detection package is a ROS package designed to detect faces and mutual gaze between the Pepper robot and a human. The package uses the media pipe library
to detect both multiple faces and their respective mutual gaze. It publishes an array of the detected faces and their mutual gaze to the topic `/face_gaze`.

## Installation 
To install the face and mutual gaze detection package, install the following packages.
First create a virtual environment and install the required packages.
```sh
cd $HOME
python3 -m venv face_gaze_detection
source face_gaze_detection/bin/activate
pip install mediapipe
```

## Running the following node to run the command
```sh
rosrun face_detection facedetection.py
```