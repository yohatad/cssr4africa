#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from retinaface.pre_trained_models import get_model

class FaceDetectionNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('face_detection_node', anonymous=True)

        # Create a CvBridge to convert ROS Image messages to OpenCV images and vice versa
        self.bridge = CvBridge()

        # Subscribe to the camera topic to receive images
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # Publisher to publish the processed images with face detection
        self.image_pub = rospy.Publisher('/face_detection/image', Image, queue_size=10)

        # Initialize RetinaFace PyTorch model (pre-trained)
        try:
            print("Loading RetinaFace model...")
            self.model = get_model("resnet50_2020-07-20", max_size=2048)
            self.model.eval()  # Set model to evaluation mode
            print("Model loaded successfully!")
        except Exception as e:
            print(f"Error loading RetinaFace model: {e}")
            self.model = None  # In case model loading fails

    def image_callback(self, data):
        if self.model is None:
            print("Model is not loaded properly. Skipping frame.")
            return

        # Convert the ROS Image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # Perform face detection using RetinaFace PyTorch model
        try:
            annotation = self.model.predict_jsons(cv_image)

            # Iterate through detected faces and draw bounding boxes and landmarks
            for face_info in annotation:
                facial_area = face_info["bbox"]  # Coordinates of the face bounding box
                landmarks = face_info["landmarks"]  # Facial landmarks (left_eye, right_eye, nose, mouth_left, mouth_right)

                # Draw bounding box around the face
                cv2.rectangle(cv_image, (facial_area[0], facial_area[1]), (facial_area[2], facial_area[3]), (0, 255, 0), 2)

                # Draw landmarks on the face
                for landmark_key, landmark in landmarks.items():
                    cv2.circle(cv_image, tuple(landmark), 5, (0, 0, 255), -1)

            # Show the image with face detection (for debugging purposes)
            cv2.imshow("RetinaFace Detection", cv_image)
            cv2.waitKey(1)

            # Convert the OpenCV image back to a ROS Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')

            # Publish the processed image with the face detection results
            self.image_pub.publish(processed_image_msg)
        except Exception as e:
            print(f"Error during face detection: {e}")

if __name__ == '__main__':
    try:
        # Instantiate the FaceDetectionNode class and spin ROS
        FaceDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
