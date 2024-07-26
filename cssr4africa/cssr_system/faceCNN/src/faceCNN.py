#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from facenet_pytorch import MTCNN
import numpy as np

# Deep SORT imports
from deep_sort_realtime.deepsort_tracker import DeepSort

class FaceDetectionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.detector = MTCNN(keep_all=True)
        
        # Initialize Deep SORT
        self.deepsort = DeepSort(max_age=100, n_init=5, nms_max_overlap=1.0, max_iou_distance=0.7)
        # Initialize ROS node
        rospy.init_node('face_detection_node', anonymous=True)

        # Subscribe to the /naoqi_driver/camera/front/image_raw topic
        self.image_sub = rospy.Subscriber('/naoqi_driver/camera/front/image_raw', Image, self.image_callback)

        # Create a publisher to publish processed images
        self.image_pub = rospy.Publisher('/face_detection', Image, queue_size=10)

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Perform face detection
            boxes, probs, landmarks = self.detector.detect(cv_image, landmarks=True)

            # Prepare detections for Deep SORT ([left, top, width, height], confidence, detection_class)
            detections = []
            if boxes is not None:
                for box, prob in zip(boxes, probs):
                    x1, y1, x2, y2 = box
                    width = x2 - x1
                    height = y2 - y1
                    detections.append(([x1, y1, width, height], float(prob), 0))  # Assuming class '0' for faces
            
            # Update Deep SORT tracker
            tracked_objects = self.deepsort.update_tracks(detections, frame=cv_image)

            # Draw tracked objects
            for track in tracked_objects:
                if not track.is_confirmed() or track.time_since_update > 1:
                    continue
                bbox = track.to_ltrb()
                track_id = track.track_id
                cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 255, 0), 2)
                cv2.putText(cv_image, f'ID: {track_id}', (int(bbox[0]), int(bbox[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Draw landmarks if detected
                if landmarks is not None:
                    for landmark in landmarks:
                        cv2.circle(cv_image, (int(landmark[0][0]), int(landmark[0][1])), 2, (255, 0, 0), 2)
                        cv2.circle(cv_image, (int(landmark[1][0]), int(landmark[1][1])), 2, (255, 0, 0), 2)

            # Display the image with bounding boxes and landmarks
            cv2.imshow('Face Detection and Tracking', cv_image)
            cv2.waitKey(1)

            # Convert OpenCV image back to ROS Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')

            # Publish the processed image
            self.image_pub.publish(processed_image_msg)
        
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

if __name__ == '__main__':
    try:
        FaceDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
