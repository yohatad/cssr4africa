#!/usr/bin/env python3

"""
face_detection_application.py Application code to run the face and mutual gaze detection algorithm.

Author: Yohannes Tadesse Haile
Date: February 15, 2024
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

    Configuration File Parameters
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
    Topic Name                              Message Type
    /camera/color/image_raw                 sensor_msgs/Image              
    /camera/depth/image_rect_raw            sensor_msgs/Image
    /naoqi_driver/camera/front/image_raw    sensor_msgs/Image
    /naoqi_driver/camera/depth/image_raw    sensor_msgs/Image

Published Topics
    Topic Name                              Message Type
    /faceDetection/data                     face_detection/face_detection.msg

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
Date: February 15, 2024
Version: v1.0
"""

import rospy
from face_detection_implementation import MediaPipe, SixDrepNet, FaceDetectionNode

def main():
    # Define the node name and software version
    node_name = "face_detection"
    software_version = " v1.0"  # Replace with the actual software version

    # Construct the copyright message
    copyright_message = (
        f"{node_name}  {software_version}\n"
        "This project is funded by the African Engineering and Technology Network (Afretec)\n"
        "Inclusive Digital Transformation Research Grant Programme.\n"
        "Website: www.cssr4africa.org\n"
        "This program comes with ABSOLUTELY NO WARRANTY."
    )
    rospy.init_node(node_name, anonymous=True)
    
    # Print the messages using ROS logging
    rospy.loginfo(copyright_message)
    rospy.loginfo(f"{node_name}: startup.")
    
    config = FaceDetectionNode.read_json_file()
    
    # set the configuration parameters to the ROS parameter server
    rospy.set_param('/faceDetection_config', config)

    algorthim = rospy.get_param('faceDetection_config/algorithm', default="sixdrep")

    if algorthim == 'mediapipe':
        face_detection = MediaPipe()
        face_detection.spin()
    elif algorthim == 'sixdrep':
        face_detection = SixDrepNet()
        face_detection.spin()
    else:
        rospy.logerr("Invalid algorithm selected. Exiting...")
        rospy.signal_shutdown("Invalid algorithm selected")

if __name__ == '__main__':
    main()