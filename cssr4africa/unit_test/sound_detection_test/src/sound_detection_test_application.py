#!/usr/bin/env python3

import rospy
from sound_detection_test_implementation import SoundDetectionTest

def main():
    node_name = "soundDetectionTest"
    software_version = "v1.0"  # Replace with the actual software version

    # Construct the copyright message
    copyright_message = (
        f"{node_name}  {software_version}\n"
        "\t\t\t    This project is funded by the African Engineering and Technology Network (Afretec)\n"
        "\t\t\t    Inclusive Digital Transformation Research Grant Programme.\n"
        "\t\t\t    Website: www.cssr4africa.org\n"
        "\t\t\t    This program comes with ABSOLUTELY NO WARRANTY."
    )

     # Initialize the ROS node
    rospy.init_node(node_name, anonymous=True)

    # Print the messages using ROS logging
    rospy.loginfo(copyright_message)
    rospy.loginfo(f"{node_name}: startup.")

    # Create the test instance and run tests
    SoundDetectionTest()

    # IMPORTANT: Keep the node alive so callbacks can be triggered
    rospy.spin()

if __name__ == "__main__":
    main()