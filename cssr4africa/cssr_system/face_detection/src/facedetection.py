#!/usr/bin/env python

import numpy as np
import cv2
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Initialize Mediapipe
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(max_num_faces=10, min_detection_confidence=0.5, min_tracking_confidence=0.5)

mp_drawing = mp.solutions.drawing_utils
drawing_spec = mp_drawing.DrawingSpec(color=(128, 0, 128), thickness=2, circle_radius=1)

# ROS Node
class MediapipeFacePoseNode:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        self.pub_pose = rospy.Publisher("/head_pose", Image, queue_size=10)
    
    def image_callback(self, data):
        # Convert ROS image to OpenCV format
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        start = rospy.get_time()

        # Flip image for selfie view
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        image.flags.writeable = False

        # Process with Mediapipe
        results = face_mesh.process(image)

        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        img_h, img_w, img_c = image.shape

        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
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

                x = angles[0] * 360
                y = angles[1] * 360
                z = angles[2] * 360

                # Determine head pose
                if y < -10:
                    text = "Looking Left"
                elif y > 10:
                    text = "Looking Right"
                elif x < -10:
                    text = "Looking Down"
                elif x > 10:
                    text = "Looking Up"
                else:
                    text = "Forward"

                # Draw nose projection
                nose_3d_projection, jacobian = cv2.projectPoints(nose_3d, rotation_vec, translation_vec, cam_matrix, distortion_matrix)

                p1 = (int(nose_2d[0]), int(nose_2d[1]))
                p2 = (int(nose_2d[0] + y * 10), int(nose_2d[1] - x * 10))

                cv2.line(image, p1, p2, (255, 0, 0), 3)
                cv2.putText(image, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)

                end = rospy.get_time()
                totalTime = end - start
                fps = 1 / totalTime
                cv2.putText(image, f'FPS: {int(fps)}', (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)

                # Draw landmarks on image
                mp_drawing.draw_landmarks(
                    image=image,
                    landmark_list=face_landmarks,
                    connections=mp_face_mesh.FACEMESH_CONTOURS,
                    landmark_drawing_spec=drawing_spec,
                    connection_drawing_spec=drawing_spec)

        # Display image using OpenCV
        cv2.imshow('Head Pose Detection', image)
        if cv2.waitKey(5) & 0xFF == 27:
            rospy.signal_shutdown("User requested shutdown")
            cap.release()

        # Publish processed image with annotations to ROS
        self.pub_pose.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

if __name__ == '__main__':
    rospy.init_node('mediapipe_face_pose', anonymous=True)
    mp_node = MediapipeFacePoseNode()
    rospy.spin()
    cv2.destroyAllWindows()
