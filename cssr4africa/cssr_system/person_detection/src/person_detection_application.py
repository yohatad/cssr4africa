#!/usr/bin/env python3
"""
person_detection_application.py Application code to run the Person Detection and Localization ROS node.

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

"""
The person detection is implemented using the ROS image topic that could be configured to be the intel realsense camera or pepper robot
camera. It uses OpenCV to visualize the detected persons. The code utilizes YOLOv8 for person detection.This code contains the main function 
that initializes the person detection node and starts the person detection algorithm. It subscribes to the intel realsense camera or pepper 
robot camera topics for the RGB and depth images.It publishes one topic: /personDetection/data that contains the person label ID, the centroid 
of the person, width and height of the boudning box.

Libraries
    - cv2
    - numpy
    - rospy
    - rospkg
    - os
    - onnxruntime
    - multiprocessing
    - json
    - random
    - lap
    - sensor_msgs.msg (Image, CompressedImage)
    - filterpy.kalman (KalmanFilter)
    - itertools (count)
    - cv_bridge (CvBridge, CvBridgeError)
    - message_filters (ApproximateTimeSynchronizer, Subscriber)
    - geometry_msgs.msg (Point)
    - cssr_system.msg (person_detection_msg_file)
    - person_detection_tracking (Sort)
    
Parameters 
    None

Configuration File Parameters
    Key                             Value
    use_compressed                  true
    confidence_iou_threshold        0.8
    sort_max_disappeared            30
    sort_min_hits                   20
    sort_iou_threshold              0.3
    verbose_mode                    true

Subscribed Topics
    Topic Name                                  Message Type
    /camera/color/image_raw                     sensor_msgs/Image              
    /camera/aligned_depth_to_color/image_raw    sensor_msgs/Image
    /naoqi_driver/camera/front/image_raw        sensor_msgs/Image
    /naoqi_driver/camera/depth/image_raw        sensor_msgs/Image

Published Topics
    Topic Name                                  Message Type
    /personDetection/data                       person_detection/person_detection_msg_file.msg

Input Data Files
    - pepperTopics.dat: Data file for Pepper robot camera topics

Model Files
    - person_detection_yolov8s.onnx: YOLOv8 model for object detection tailored for person detection

Output Data Files
    None

Configuration File
    person_detection_configuration.json

Example of instantiation of the module
    roslauch cssr_system person_detection_robot.launch camera:=realsense

    (In a new terminal)
    rosrun cssr_system person_detection_application.py

Author: Yohannes Tadesse Haile, Carnegie Mellon University Africa
Email: yohanneh@andrew.cmu.edu
Date: March 28, 2025
Version: v1.0
"""

import rospy
from person_detection_implementation import PersonDetectionNode, YOLOv8

def main():
    # Define the node name and software version
    node_name = "person_detection"
    software_version = "v1.0"  # Replace with the actual software version
    
    # Construct the copyright message
    copyright_message = (
        f"{node_name} {software_version}\n"
        "This project is funded by the African Engineering and Technology Network (Afretec)\n"
        "Inclusive Digital Transformation Research Grant Programme.\n"
        "Website: www.cssr4africa.org\n"
        "This program comes with ABSOLUTELY NO WARRANTY."
    )
    
    rospy.init_node(node_name, anonymous=True)
    
    # Print the messages using ROS logging
    rospy.loginfo(copyright_message)
    rospy.loginfo(f"{node_name}: startup.")
    
    # Read the configuration file
    config = PersonDetectionNode.read_json_file('cssr_system')
    
    unit_test = rospy.get_param('/personDetection/unit_test', default=False)
    
    if not unit_test:
        rospy.set_param('/personDetection_config', config)
    else:
        # Create a filtered config without the excluded keys
        filtered_config = {k: v for k, v in config.items() 
                        if k not in ["use_compressed", "verbose_mode"]}
        
        # Set the filtered parameters to the parameter server
        for key, value in filtered_config.items():
            rospy.set_param('/personDetection_config/' + key, value)

        rospy.set_param('/personDetection_config/' + key, value)

        # Set the algorthim, use_compressed, and verbose_mode parameters
        config_test = PersonDetectionNode.read_json_file('unit_test')
        
        # Filter and set only the specific parameters from the test config
        for key, value in config_test.items():
            if key in ["use_compressed", "algorithm", "verbose_mode"]:
                rospy.set_param('/personDetection_config/' + key, value)
    
    person_detection = YOLOv8()
    person_detection.spin()

if __name__ == '__main__':
    main()