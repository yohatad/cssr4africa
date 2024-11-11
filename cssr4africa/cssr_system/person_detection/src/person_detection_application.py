import rospy
from person_detection_implementation import PersonDetection, YOLOv8ROS

def main():
    rospy.init_node('person_detection_application')
    person_detection = PersonDetection(YOLOv8ROS())
    person_detection.spin()

if __name__ == '__main__':
    main()
