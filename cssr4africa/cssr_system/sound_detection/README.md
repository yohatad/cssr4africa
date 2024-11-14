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
sudo apt install python3.8-venv
python3.8 -m venv testdsound
source testdsound/bin/activate
pip install torch==1.13.1+cu117 torchvision==0.14.1+cu117 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu117
pip install scipy soundfile webrtcvad pyYaml onnxruntime rospkg rospy
```

## Download the model
You need to download the [NSNet](https://drive.google.com/file/d/1G8OPmx8RlrEbagMmK0-OG023XqB45ta_/view?usp=sharing). Put them in the models folder of the sound detection package. 

Use the launch file of the pepper_interface_package and set the right network_interface. 
1. roslaunch pepper_interface_tests sensorTestLaunchRobot.launch network_interface:=wlp0s20f3

2. Then run the sound detection and localization.
    ```sh
    rosrun sound_detection sound_detection_application.py
    ```

## Output
The node publishes the sound signal in the **soundDetection/signal** and the angle of the direction of the sound in the **soundDetection/direction**.

## License
Copyright (C) 2023 CSSR4Africa Consortium  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme