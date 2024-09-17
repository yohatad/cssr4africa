#!/usr/bin/env python

import time
import cv2 as cv
import mediapipe as mp
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Initialize Mediapipe for face detection
mp_face_detection = mp.solutions.face_detection

# ROS Node
class MediapipeFaceDetectionNode:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        self.face_detector = mp_face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5)
        self.frame_counter = 0
        self.start_time = time.time()

    def image_callback(self, data):
        # Convert ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        self.frame_counter += 1
        rgb_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

        # Process the image with Mediapipe Face Detection
        results = self.face_detector.process(rgb_frame)
        frame_height, frame_width, _ = frame.shape
        
        # If faces are detected, draw bounding boxes and key points
        if results.detections:
            for face in results.detections:
                face_rect = np.multiply(
                    [
                        face.location_data.relative_bounding_box.xmin,
                        face.location_data.relative_bounding_box.ymin,
                        face.location_data.relative_bounding_box.width,
                        face.location_data.relative_bounding_box.height,
                    ],
                    [frame_width, frame_height, frame_width, frame_height]
                ).astype(int)

                # Draw bounding box
                cv.rectangle(frame, face_rect, color=(255, 255, 255), thickness=2)

                # Extract and draw key points
                key_points = np.array([(p.x, p.y) for p in face.location_data.relative_keypoints])
                key_points_coords = np.multiply(key_points, [frame_width, frame_height]).astype(int)
                for p in key_points_coords:
                    cv.circle(frame, tuple(p), 4, (255, 255, 255), 2)
                    cv.circle(frame, tuple(p), 2, (0, 0, 0), -1)

        # Calculate and display FPS
        fps = self.frame_counter / (time.time() - self.start_time)
        cv.putText(frame, f"FPS: {fps:.2f}", (30, 30), cv.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 255), 2)

        # Display the frame using OpenCV
        cv.imshow("Face Detection", frame)
        key = cv.waitKey(1)
        if key == ord("q"):
            rospy.signal_shutdown("User requested shutdown")

if __name__ == '__main__':
    rospy.init_node('mediapipe_face_detection', anonymous=True)
    mp_node = MediapipeFaceDetectionNode()
    rospy.spin()
    cv.destroyAllWindows()
