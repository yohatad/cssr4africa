#!/usr/bin/env python

"""
face_detection_application.py Application code to run the face and mutual gaze detection algorithm.

Author: Yohannes Tadesse Haile
Date: December 15, 2024
Version: v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

"""
face_detection_application.py   Application code to run the face and mutual gaze detection algorithm.

The face detectionis implemented using the ROS image topic that could be configured to be the intel realsense camera or pepper robot
camera. It uses OpenCV to visualize the detected faces and gaze direction. The gaze direction is calculated using face
mesh landmarks which uses Google's MediaPipe library. The media pipe utilizes CPU for face detection and gaze direction.
The SixDrepNet uses YOLOONNX for face detection and SixDrepNet for gaze direction. The SixDrepNet utilizes GPU for faster
inference and better performance. This code contains the main function that initializes the face detection node and 
starts the face detection algorithm. The face detection algorithm can be either MediaPipe Face Detection or SixDrepNet 
that can be configured from the configuration file. It is also responsible for detecting the head pose esimation of the 
detected face. It subscribes to the intel realsense camera or pepper robot camera topics for the RGB and depth images.
It publishes three one topic: /faceDetection/data that contains the face label ID, the centroid of the face, 
mutual gaze direction. 

Libraries
    - cv2: OpenCV library for image processing
    - mediapipe: Google's MediaPipe library for face detection and gaze direction
    - numpy: NumPy library for numerical operations
    - rospy: ROS Python library for ROS operations
    - rospkg: ROS Python library for ROS package operations
    - os: Python library for operating system operations
    - onnxruntime: ONNX Runtime library for deep learning model inference
    - multiprocessing: Python library for multiprocessing operations
    - collections: Python library for collection operations
    - scipy.spatial: SciPy library for spatial operations
    - distance: SciPy library for distance operations
    - Point: ROS geometry_msgs library for Point operations
    - faceDetection: ROS face_detection library for face detection operations

Parameters
    Command line arguments: None

Configuration file parameters

    Key                             Value
    camera                          realsense
    algorithm                       mediapipe
    centroid_max_distance           15
    centroid_max_disappeared        100
    mp_facedet_confidence           0.5
    mp_headpose_angle               5
    sixdrepnet_confidence           0.65
    sixdrepnet_headpose_angle       10
    verboseMode                     True

Subscribed Topics
    RealSenseCameraRGB      /camera/color/image_raw
    RealSenseCameraDepth    /camera/depth/image_rect_raw
    PepperFrontCamera       /naoqi_driver/camera/front/image_raw
    PepperDepthCamera       /naoqi_driver/camera/depth/image_raw

Published Topics
    /faceDetection/data

Input Data Files
    - faceDetectionConfiguration.ini: Configuration file for face detection parameters
    - pepperTopics.dat: Data file for Pepper robot camera topics
    - face_detection_YOLO.onnx: YOLOONNX model for face detection
    - face_detection_sixdrepnet360.onnx: SixDrepNet model for gaze direction

Output Data Files
    None

Configuration File
    face_detection_configuration.ini

Example of instantiation of the module
    rosrun face_detection face_detection_application.py

Author: Yohannes Tadesse Haile
Email: yohanneh@andrew.cmu.edu
Date: December 15, 2024
Version: v1.0
"""

import rospy
from face_detection_implementation import MediaPipeFaceNode, SixDrepNet, FaceDetectionNode

def main():
    rospy.init_node('face_detection', anonymous=True)
    
    config = FaceDetectionNode.parse_config()
    algorthim = config.get('algorithm', 'mediapipe')

    if algorthim == 'mediapipe':
        face_detection = MediaPipeFaceNode(config)
        face_detection.spin()
    elif algorthim == 'sixdrep':
        face_detection = SixDrepNet(config)
        face_detection.spin()
    else:
        rospy.logerr("Invalid algorithm selected. Exiting...")
        rospy.signal_shutdown("Invalid algorithm selected")

if __name__ == '__main__':
    main()
