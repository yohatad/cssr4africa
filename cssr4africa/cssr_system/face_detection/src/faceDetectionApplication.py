#!/usr/bin/env python

"""
faceDetectionApplication.py

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY

<detailed functional description of the program>
This code containes the main function that initializes the face detection node and starts the face detection algorithm
The face detection algorithm can be either MediaPipe Face Detection or SixDrepNet that can be configured from the 
configuration file. It is also reponsible for detecting the head pose esimation of the detected face. It publishes three 
one topic: /faceDetection/data that contains the face label ID, the centorid of the face, mutual gaze direction. 

Libraries:
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

Parameters:
    Command line arguments: None

Configuration file parameters:

    Key                             Value
    algorithm                       mediapipe
    camera                          realsense
    centroid_max_distance           15
    centroid_max_disappeared        100
    mp_facedet_confidence           0.5
    mp_headpose_angle               5
    sixdrepnet_confidence           0.65
    sixdrepnet_headpose_angle       10
    deepsort_max_age                7
    deepsort_max_iou_distance       0.7
    deepsort_n_init                 2
    verboseMode                     True

Subscribed Topics:
    RealSenseCameraRGB      /camera/color/image_raw
    RealSenseCameraDepth    /camera/depth/image_rect_raw
    PepperFrontCamera       /naoqi_driver/camera/front/image_raw

Published Topics:
    /faceDetection/data

Input Data Files:
    - faceDetectionConfiguration.ini: Configuration file for face detection parameters
    - pepperTopics.dat: Data file for Pepper robot camera topics
    - gold_yolo_n_head_post_0277_0.5071_1x3x480x640: YOLOONNX model for face detection
    - sixdrepnet360_Nx3x224x224: SixDrepNet model for gaze direction

Output Data Files: 
    None

Configuration File:
    faceDetectionConfiguration.ini

Example of instantiation of the module:
    rosrun face_detection faceDetectionApplication.py


Author: Yohannes Tadesse Haile
Email: yohanneh@andrew.cmu.edu
Date: November 1, 2024
Version: v1.0
"""

import rospy
from faceDetectionImplementation import MediaPipeFaceNode, SixDrepNet, FaceDetectionNode

def main():
    rospy.init_node('face_detection_node', anonymous=True)
    
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
