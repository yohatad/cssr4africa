#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

# Initialize YOLOv8 model
model = YOLO('yolov8n.pt')  # You can use any model variant like 'yolov8s.pt' for better accuracy

class YOLOPersonDetector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolov8_person_detector', anonymous=True)
        
        # Create a subscriber to the camera image topic
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        
        # Initialize the CvBridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()
        
        # Open a window to display the results
        cv2.namedWindow("YOLOv8 Person Detection", cv2.WINDOW_NORMAL)

    def image_callback(self, img_msg):
        try:
            # Convert the image from ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
            return
        
        # Run YOLOv8 detection
        results = model(frame)

        # Filter results to show only 'person' class detections (class ID 0 in YOLO's COCO dataset)
        for detection in results[0].boxes:
            if detection.cls == 0:  # Class 0 is 'person' in YOLOv8's COCO dataset
                # Get bounding box coordinates
                x1, y1, x2, y2 = map(int, detection.xyxy[0].cpu().numpy())
                conf = detection.conf.item()  # Get confidence level
                
                # Draw the bounding box and confidence score on the image
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'Person {conf:.2f}'
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the image with detections
        cv2.imshow("YOLOv8 Person Detection", frame)
        cv2.waitKey(1)  # Display the image for 1 ms and move to the next frame

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        person_detector = YOLOPersonDetector()
        person_detector.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
