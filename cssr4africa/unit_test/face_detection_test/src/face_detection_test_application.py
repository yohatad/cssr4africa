#!/usr/bin/env python

"""
face_detection_application.py Application code to run the face and mutual gaze detection algorithm.

Author: Yohannes Tadesse Haile
Date: December 15, 2024
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
    test = FaceDetectionTest()
    test.run_tests()

    # IMPORTANT: Keep the node alive so callbacks can be triggered
    rospy.spin()

if __name__ == '__main__':
    main()
