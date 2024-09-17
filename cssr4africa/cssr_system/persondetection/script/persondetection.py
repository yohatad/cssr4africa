#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import String

class YOLOv8ROS:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('yolov8_node', anonymous=True)

        # Define YOLOv8 model
        self.yolo_model = YOLO('/root/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/persondetection/script/yolov8s.pt')

        # ROS topics and publishers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  # Subscribing to camera/color/image_raw
        self.person_detected_pub = rospy.Publisher('/yolov8/person_detected', String, queue_size=10)
        
        # Initialize OpenCV bridge for image conversion
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run YOLOv8 detection on the frame
        results = self.yolo_model(frame)

        # Extract bounding boxes and confidences for persons
        person_detected = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
                conf = float(box.conf[0])  # Confidence score
                class_id = int(box.cls[0])  # Class ID of the detected object

                # Only process detections for class "person" (COCO class 0) with confidence above threshold
                if class_id == 0 and conf > 0.5:
                    # Store detected person information
                    person_detected.append(f'Bbox: [{x1}, {y1}, {x2}, {y2}], Conf: {conf:.2f}')

                    # Draw bounding box and confidence score on the image
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f'Conf: {conf:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # Publish detected person information
        self.person_detected_pub.publish(str(person_detected))

        # Display the processed frame (optional)
        cv2.imshow("YOLOv8 ROS", frame)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        yolov8_ros = YOLOv8ROS()
        yolov8_ros.run()
    except rospy.ROSInterruptException:
        pass
