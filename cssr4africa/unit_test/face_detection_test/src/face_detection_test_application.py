#!/usr/bin/env python3

"""
face_detection_test_application.py Application code to run the Face and Mutual Gaze Detection and Localization Unit test.

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

"""
face_detection_test_application.py  Application code to run the Face and Mutual Gaze Detection and Localization unit test.

This face_detection_test is a unit test application code to test the face and mutual gaze detection algorithm.
This code contains the main function that initializes the correct configuration parameters and tests face 
detection algorthim. It has also utility functions to save video and images with the bounding boxes and gaze 
detection. The face detection algorithm can be either MediaPipe Face Detection or SixDrepNet that can be
configured from the configuration file. 

Libraries
    - rospkg
    - rospy
    - os
    - json
    - numpy
    - cv2
    - time
    - threading
    - colorsys
    - sensor_msgs.msg (Image)
    - cv_bridge (CvBridge)
    - message_filters (ApproximateTimeSynchronizer, Subscriber)
    - face_detection_test.msg (msg_file)

Parameters
    Launch File Parameters:
        roslaunch unit_test face_detection_test_launch_robot.launch camera:=realsense
            camera: Camera type or video file (realsense or pepper or video)
            bag_file: ROS bag file for testing (singleFace, multipleFaces, faceTracking, mutualGaze, occlusion, lighting)
            pepper_robot_ip: Pepper robot IP address (e.g 172.29.111.230 or 172.29.111.240)
            network_interface: Network interface for Pepper robot connection

        roslaunch unit_test face_detection_test_launch_testHarness.launch

    Configuration File Parameters
        Key                             Value
        algorithm                       sixdrep
        save_video                      false
        save_image                      false
        video_duration                  10
        image_interval                  5
        recording_delay                 5
        max_frames_buffer               300
        verbose_mode                    false

Subscribed Topics
    Topic Name                                  Message Type
    /camera/color/image_raw                     sensor_msgs/Image              
    /camera/aligned_depth_to_color/image_raw    sensor_msgs/Image
    /naoqi_driver/camera/front/image_raw        sensor_msgs/Image
    /naoqi_driver/camera/depth/image_raw        sensor_msgs/Image
    /faceDetection/data                         face_detection/face_detection.msg

Published Topics
    None

Input Data Files
    - pepperTopics.dat: Data file for Pepper robot camera topics
    - face_detection_test_input_realsense_singleFace.bag
    - face_detection_test_input_realsense_multipleFaces.bag
    - face_detection_test_input_realsense_faceTracking.bag
    - face_detection_test_input_realsense_mutualGaze.bag
    - face_detection_test_input_realsense_occlusion.bag
    - face_detection_test_input_realsense_lighting.bag

Output Data Files
    - face_detection_rgb_video_{start_time}.mp4
    - face_detection_depth_video_{start_time}.mp4
    - face_detection_rgb_image_{start_time}.png
    - face_detection_depth_image_{start_time}.png

Configuration File
    face_detection_test_configuration.json

Example of instantiation of the module
    roslaunch unit_test face_detection_test_launch_robot.launch camera:=video bag_file:=singleFace
    
    (In a new terminal)
    roslaunch unit_test face_detection_test_launch_testHarness.launch

Author: Yohannes Tadesse Haile
Email: yohanneh@andrew.cmu.edu
Date: March 21, 2025
Version: v1.0
"""

import rospy
from face_detection_test_implementation import FaceDetectionTest

def main():
    # Define the node name and software version
    node_name = "face_detection_test"
    software_version = "v1.0"  # Replace with the actual software version

    # Construct the copyright message
    copyright_message = (
        f"{node_name}  {software_version}\n"
        "This project is funded by the African Engineering and Technology Network (Afretec)\n"
        "Inclusive Digital Transformation Research Grant Programme.\n"
        "Website: www.cssr4africa.org\n"
        "This program comes with ABSOLUTELY NO WARRANTY."
    )

    # Initialize the ROS node
    rospy.init_node(node_name, anonymous=True)

    # Print the messages using ROS logging
    rospy.loginfo(copyright_message)
    rospy.loginfo(f"{node_name}: startup.")

    # Create the test instance and run tests
    FaceDetectionTest()

    # IMPORTANT: Keep the node alive so callbacks can be triggered
    rospy.spin()

if __name__ == '__main__':
    main()
