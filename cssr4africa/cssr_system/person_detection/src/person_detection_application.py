#!/usr/bin/env python3

import rospy
from person_detection_implementation import PersonDetectionNode, YOLOv8ROS

def main():
    rospy.init_node('person_detection_application')

    config = PersonDetectionNode.read_json_file()

    # Set the configuration parameters to the ROS parameter server
    rospy.set_param('/personDetection_config', config)
    person_detection = YOLOv8ROS()
    person_detection.spin()

if __name__ == '__main__':
    main()
