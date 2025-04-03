#!/usr/bin/env python3

"""
sound_localization_application.py

Application code to run the sound detection and localization algorithm.

Author: Yohannes Tadesse Haile
Date: February 15, 2025
Version: v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

"""
sound_localization_application.py   Application code to run the sound detection and localization algorithm.

The sound localization algorithm is implemented using a ROS audio topic that can be configured to receive audio from
a robot or an external microphone. It processes the incoming audio signal to detect sound events and localize the sound
source by computing the interaural time difference (ITD) using the GCC-PHAT method. The ITD is then converted into an
angle of arrival using physical parameters such as the speed of sound and the distance between the microphones.
This code contains the main function that initializes the sound localization node, loads the configuration, and starts
the algorithm. The algorithm is designed to accumulate fixed-size audio samples (e.g., 4096 per callback) in a rolling
buffer until sufficient data is collected for processing.

Libraries:
    - math: Provides mathematical functions for calculations.
    - numpy: Used for numerical operations and array manipulations.
    - rospy: ROS Python library for node initialization and message handling.
    - rospkg: ROS library for package management and locating configuration files.
    - os: Interface for operating system functionalities.
    - json: Handles configuration file reading in JSON format.
    - webrtcvad: Implements Voice Activity Detection (VAD) for sound segmentation.
    - std_msgs: Contains standard ROS message types.
    - threading: Provides thread locking for safe concurrent operations.

Parameters:
    Command line arguments: None

    Configuration File Parameters:
        Key                             Value
        intensity_threshold             [float]     e.g., 3.9e-3
        vad_aggressiveness              [int]       e.g., 3
        sample_rate                     [int]       e.g., 48000
        localization_buffer_size        [int]       e.g., 8192

Subscribed Topics:
    Topic Name                              Message Type
    /naoqi_driver/audio                     sound_detection/sound_detection

Published Topics:
    Topic Name                              Message Type
    /soundDetection/signal                   std_msgs/Float32MultiArray
    /soundDetection/direction                std_msgs/Float32

Input Data Files:
    - pepper_topics.dat: Data file containing topic names for the robot's audio sources.

Output Data Files:
    None

Configuration File:
    sound_detection_configuration.json

Example of instantiation of the module:
    rosrun sound_detection sound_localization_application.py

Author: Yohannes Tadesse Haile
Email: yohanneh@andrew.cmu.edu
Date: February 15, 2025
Version: v1.0

"""

import rospy
from sound_detection_implementation import SoundDetectionNode 

def main():
    # Define the node name and software version
    node_name = "soundDetection"
    software_version = " v1.0"  # Replace with the actual software version

    # Construct the copyright message
    copyright_message = (
        f"{node_name}  {software_version}\n"
        "This project is funded by the African Engineering and Technology Network (Afretec)\n"
        "Inclusive Digital Transformation Research Grant Programme.\n"
        "Website: www.cssr4africa.org\n"
        "This program comes with ABSOLUTELY NO WARRANTY."
    )
    
    # Initialize the ROS node.
    rospy.init_node('sound_detection_node', anonymous=True)

    # Print the messages using ROS logging
    rospy.loginfo(copyright_message)
    rospy.loginfo(f"{node_name}: startup.")
    

    # Create an instance of your node.
    node = SoundDetectionNode()
        
    # Run the node.
    node.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
