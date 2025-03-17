#!/usr/bin/env python3

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
    config = PersonDetectionNode.read_json_file()
    
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
    
    person_detection = YOLOv8()
    person_detection.spin()

if __name__ == '__main__':
    main()