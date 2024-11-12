#!/usr/bin/env python3

import rospy
from person_detection_implementation import PersonDetectionNode, YOLOv8ROS

def main():
    rospy.init_node('person_detection_application')

    config = PersonDetectionNode.parse_config()

    person_detection = YOLOv8ROS(config)
    
    person_detection.spin()

if __name__ == '__main__':
    main()
