#!/usr/bin/env python3
import rospy
import mediapipe as mp
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from skeletal_model_retargeting_implementation import HumanToPepperRetargeting

def pixel_to_3d_camera_coords(x, y, depth, K_matrix):
    point_2d_hom = np.array([x, y, 1])
    K_matrix_inv = np.linalg.inv(K_matrix)
    point_3d = depth * (K_matrix_inv @ point_2d_hom)
    return point_3d

def mid_point(P1, P2):
    return [(P1[0] + P2[0]) / 2, (P1[1] + P2[1]) / 2, (P1[2] + P2[2]) / 2]

class SkeletalModelEstimationROS:
    def __init__(self, camera_intrinsics, image_width, image_height, retargeting):
        self.intrinsics = camera_intrinsics
        self.image_width = image_width
        self.image_height = image_height
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.retargeting = retargeting
        self.bridge = CvBridge()

        # ROS subscribers
        self.color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)

        # Storage for synchronized color and depth frames
        self.color_image = None
        self.depth_image = None

    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error (color): {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error (depth): {e}")

    def process_image(self):
        if self.color_image is None or self.depth_image is None:
            return

        # Get landmarks and calculate angles
        landmarks = self.get_landmarks(self.color_image)
        if landmarks is not None:
            angles = self.get_pepper_angles(landmarks, self.depth_image)
            if angles is not None:
                rospy.loginfo("Joint Angles:")
                colors = ["[91m", "[92m", "[93m", "[94m", "[91m", "[92m", "[93m", "[94m"]
                for idx, (joint, angle) in enumerate(angles.items()):
                    color = colors[idx % len(colors)]
                    rospy.loginfo(f"{color}{joint}: {angle:.2f} degrees[0m")
        else:
            rospy.logwarn("Landmarks could not be calculated.")

    def get_landmarks(self, bgr_image):
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        rgb_image.flags.writeable = False
        results = self.mp_pose.process(rgb_image)
        if results.pose_landmarks:
            return results.pose_landmarks.landmark
        else:
            return None
        
    def checkLim(val, limits):
        return val < limits[0] or val > limits[1]

    def get_pepper_angles(self, landmarks, depth_image):
        LShoulder = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER.value], depth_image)
        RShoulder = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER.value], depth_image)
        LElbow = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.LEFT_ELBOW.value], depth_image)
        RElbow = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.RIGHT_ELBOW.value], depth_image)
        LWrist = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.LEFT_WRIST.value], depth_image)
        RWrist = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.RIGHT_WRIST.value], depth_image)
        LHip = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.LEFT_HIP.value], depth_image)
        RHip = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.RIGHT_HIP.value], depth_image)

        if any(coord is None for coord in [LShoulder, RShoulder, LElbow, RElbow, LWrist, RWrist, LHip, RHip]):
            return None

        Neck = np.array(mid_point(LShoulder, RShoulder))
        MidHip = np.array(mid_point(LHip, RHip))

        wp_dict = {
            '1': Neck,
            '2': RShoulder,
            '3': RElbow,
            '4': RWrist,
            '5': LShoulder,
            '6': LElbow,
            '7': LWrist,
            '8': MidHip
        }

        angles = self.retargeting.get_angles(wp_dict)
        return {
            'LShoulderPitch': angles[0],
            'LShoulderRoll': angles[1],
            'LElbowYaw': angles[2],
            'LElbowRoll': angles[3],
            'RShoulderPitch': angles[4],
            'RShoulderRoll': angles[5],
            'RElbowYaw': angles[6],
            'RElbowRoll': angles[7],
        }
    
    def get_3d_coordinates(self, landmark, depth_image):
        x = int(landmark.x * self.image_width)
        y = int(landmark.y * self.image_height)

        if x < 0 or x >= depth_image.shape[1] or y < 0 or y >= depth_image.shape[0]:
            return None

        z = depth_image[y, x]
        if z == 0 or np.isnan(z):
            return None

        return pixel_to_3d_camera_coords(x, y, z, self.intrinsics)

if __name__ == "__main__":
    rospy.init_node('skeletal_model_estimation_node', anonymous=True)

    intrinsics = [
        [606.12, 0.0, 321.38],
        [0.0, 605.72, 256.30],
        [0.0, 0.0, 1.0]
    ]

    # K: [909.1842651367188, 0.0, 642.069580078125, 0.0, 908.5780639648438, 384.4430236816406, 0.0, 0.0, 1.0]

    # intrinsics = [
    #     [909.1842651367188, 0.0, 642.069580078125],
    #     [0.0, 908.5780639648438, 384.4430236816406],
    #     [0.0, 0.0, 1.0]
    # ]

    retargeting = HumanToPepperRetargeting()
    skeletal_estimator = SkeletalModelEstimationROS(
        camera_intrinsics=intrinsics,
        image_width=640,
        image_height=480,
        retargeting=retargeting
    )

    rate = rospy.Rate(2)  # 10 Hz
    while not rospy.is_shutdown():
        skeletal_estimator.process_image()
        rate.sleep()
