#!/usr/bin/env python

import time
import cv2
import mediapipe as mp
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from face_detection.msg import faceDetection  # Replace with your actual package name

# Initialize Mediapipe for face mesh and face detection
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(max_num_faces=10, min_detection_confidence=0.5, min_tracking_confidence=0.5)

mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5)

mp_drawing = mp.solutions.drawing_utils
drawing_spec = mp_drawing.DrawingSpec(color=(128, 128, 128), thickness=1, circle_radius=1)


# ROS Node
class MediapipeFaceNode:
    def __init__(self):
        # ROS subscribers and publishers
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        self.pub_gaze = rospy.Publisher("/faceDetection/data", faceDetection, queue_size=10)  # Publisher for the faceDetection message

        # For FPS calculation
        self.frame_counter = 0
        self.start_time = time.time()

    def image_callback(self, data):
        # Convert ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        self.frame_counter += 1
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_h, img_w, _ = frame.shape

        # Process with face detection
        self.process_face_detection(frame, rgb_frame, img_h, img_w)

        # Process with face mesh
        self.process_face_mesh(frame, rgb_frame, img_h, img_w)

        # Calculate and display FPS
        fps = self.frame_counter / (time.time() - self.start_time)
        cv2.putText(frame, f"FPS: {fps:.2f}", (30, 30), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 255), 2)

        # Display the frame using OpenCV
        cv2.imshow("Face Detection & Mesh", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rospy.signal_shutdown("User requested shutdown")

    def process_face_detection(self, frame, rgb_frame, img_h, img_w):
        # Process the image with Mediapipe Face Detection
        results = face_detection.process(rgb_frame)
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
                cv2.rectangle(frame, face_rect, color=(255, 255, 255), thickness=2)

                # Extract and draw key points
                key_points = np.array([(p.x, p.y) for p in face.location_data.relative_keypoints])
                key_points_coords = np.multiply(key_points, [frame_width, frame_height]).astype(int)
                for p in key_points_coords:
                    cv2.circle(frame, tuple(p), 4, (255, 255, 255), 2)
                    cv2.circle(frame, tuple(p), 2, (0, 0, 0), -1)

    def process_face_mesh(self, frame, rgb_frame, img_h, img_w):
        # Process with Mediapipe Face Mesh
        results = face_mesh.process(rgb_frame)

        # Initialize arrays for centroids and forward gaze status
        centroids = []
        mutualGaze_list = []

        if results.multi_face_landmarks:
            for face_id, face_landmarks in enumerate(results.multi_face_landmarks):
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
                centroids.append(centroid)

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

                cv2.line(frame, p1, p2, (255, 0, 0), 3)

                # Display text (forward or not forward)
                text = "Forward" if mutualGaze else "Not Forward"
                label = f"Face {face_id + 1}: {text}"
                cv2.putText(frame, label, (int(centroid_x), int(centroid_y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Draw landmarks on image
                mp_drawing.draw_landmarks(
                    image=frame,
                    landmark_list=face_landmarks,
                    connections=mp_face_mesh.FACEMESH_CONTOURS,
                    landmark_drawing_spec=drawing_spec,
                    connection_drawing_spec=drawing_spec)

        # Publish centroids and mutual gaze status
        gaze_msg = faceDetection()
        gaze_msg.centroids = centroids
        gaze_msg.mutualGaze = mutualGaze_list
        self.pub_gaze.publish(gaze_msg)


if __name__ == '__main__':
    rospy.init_node('mediapipe_face_combined', anonymous=True)
    mp_node = MediapipeFaceNode()
    rospy.spin()
    cv2.destroyAllWindows()
