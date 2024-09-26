#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from facenet_pytorch import MTCNN
from geometry_msgs.msg import Point
import mediapipe as mp
import numpy as np

class FaceDetectionAndPoseEstimationNode:
    def __init__(self):
        self.bridge = CvBridge()
        
        # Initialize MTCNN for face detection
        self.mtcnn = MTCNN(keep_all=True, device= 'cuda')

        # Initialize Mediapipe Face Mesh
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(max_num_faces=10, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        
        self.mp_drawing = mp.solutions.drawing_utils
        self.drawing_spec = self.mp_drawing.DrawingSpec(color=(128, 128, 128), thickness=1, circle_radius=1)

        # Initialize ROS node
        rospy.init_node('face_detection_pose_estimation_node', anonymous=True)

        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # Publisher for processed images
        self.image_pub = rospy.Publisher('/face_detection_pose_estimation', Image, queue_size=10)

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Perform face detection using MTCNN on the entire image
            boxes, probs = self.mtcnn.detect(cv_image)

            # Proceed only if at least one face is detected
            if boxes is not None and len(boxes) > 0:
                # Optional: Draw bounding boxes around detected faces
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 155, 255), 2)

                # Flip image for selfie-view
                cv_image = cv2.cvtColor(cv2.flip(cv_image, 1), cv2.COLOR_BGR2RGB)
                cv_image.flags.writeable = False

                # Perform face mesh landmark detection using Mediapipe
                results = self.face_mesh.process(cv_image)

                cv_image.flags.writeable = True
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

                img_h, img_w, img_c = cv_image.shape

                mutualGaze_list = []

                if results.multi_face_landmarks:
                    for face_id, face_landmarks in enumerate(results.multi_face_landmarks):
                        # Initialize face_2d and face_3d as lists for each detected face
                        face_2d = []
                        face_3d = []

                        for idx, lm in enumerate(face_landmarks.landmark):
                            if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:
                                if idx == 1:
                                    nose_2d = (lm.x * img_w, lm.y * img_h)
                                    nose_3d = (lm.x * img_w, lm.y * img_h, lm.z * 3000)
                                x, y = int(lm.x * img_w), int(lm.y * img_h)

                                face_2d.append([x, y])
                                face_3d.append([x, y, lm.z])

                        # Calculate centroid
                        centroid_x = np.mean([pt[0] for pt in face_2d])
                        centroid_y = np.mean([pt[1] for pt in face_2d])
                        centroid = Point(x=centroid_x, y=centroid_y, z=0)  # z = 0 for 2D image coordinates

                        # Convert face_2d and face_3d to NumPy arrays for solvePnP
                        face_2d = np.array(face_2d, dtype=np.float64)
                        face_3d = np.array(face_3d, dtype=np.float64)

                        focal_length = 1 * img_w
                        cam_matrix = np.array([[focal_length, 0, img_h / 2],
                                            [0, focal_length, img_w / 2],
                                            [0, 0, 1]])

                        distortion_matrix = np.zeros((4, 1), dtype=np.float64)

                        success, rotation_vec, translation_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, distortion_matrix)

                        rmat, jac = cv2.Rodrigues(rotation_vec)
                        angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)

                        x_angle = angles[0] * 360
                        y_angle = angles[1] * 360
                        z_angle = angles[2] * 360

                        # Determine if head is facing forward
                        mutualGaze = abs(x_angle) <= 5 and abs(y_angle) <= 5
                        mutualGaze_list.append(mutualGaze)

                        # Draw nose projection
                        nose_3d_projection, jacobian = cv2.projectPoints(nose_3d, rotation_vec, translation_vec, cam_matrix, distortion_matrix)

                        p1 = (int(nose_2d[0]), int(nose_2d[1]))
                        p2 = (int(nose_2d[0] + y_angle * 10), int(nose_2d[1] - x_angle * 10))

                        cv2.line(cv_image, p1, p2, (255, 0, 0), 3)

                        # Display text (forward or not forward)
                        text = "Forward" if mutualGaze else "Not Forward"
                        label = f"Face {face_id + 1}: {text}"
                        cv2.putText(cv_image, label, (int(centroid_x), int(centroid_y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                        # Draw landmarks on image
                        self.mp_drawing.draw_landmarks(
                            image=cv_image,
                            landmark_list=face_landmarks,
                            connections=self.mp_face_mesh.FACEMESH_CONTOURS,
                            landmark_drawing_spec=self.drawing_spec,
                            connection_drawing_spec=self.drawing_spec)

            else:
                # No face detected by MTCNN, do not run face mesh
                pass

            # Display the image with face detection, pose estimation, and the blue line
            cv2.imshow('Face Detection and Pose Estimation', cv_image)
            cv2.waitKey(1)

            # Convert OpenCV image back to ROS Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')

            # Publish the processed image
            self.image_pub.publish(processed_image_msg)

        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def shutdown_hook(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = FaceDetectionAndPoseEstimationNode()
        rospy.on_shutdown(node.shutdown_hook)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
