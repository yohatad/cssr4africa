#!/usr/bin/env python3
import rospy
import mediapipe as mp
import cv2
import numpy as np
import time
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
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_pose = mp.solutions.pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.retargeting = retargeting
        self.bridge = CvBridge()

        self.last_logged_time = rospy.get_time()

        # ROS subscribers
        self.color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        
        # ROS publishers for visualization
        self.visualization_pub = rospy.Publisher("/skeletal_model/visualization", Image, queue_size=10)
        self.skeleton_3d_pub = rospy.Publisher("/skeletal_model/skeleton_3d_vis", Image, queue_size=10)

        # Storage for synchronized color and depth frames
        self.color_image = None
        self.depth_image = None
        
        # Dictionary to store 3D coordinates of landmarks
        self.landmarks_3d = {}
        
        # Create a figure for 3D visualization
        self.fig3d = None
        self.ax3d = None
        try:
            from matplotlib import pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
            plt.ion()
            self.fig3d = plt.figure(figsize=(10, 8))
            self.ax3d = self.fig3d.add_subplot(111, projection='3d')
            self.plt = plt
        except ImportError:
            rospy.logwarn("Matplotlib not available. 3D visualization will be disabled.")

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

        # Create a copy for visualization
        viz_image = self.color_image.copy()
        
        # Get landmarks and calculate angles
        landmarks, results = self.get_landmarks(self.color_image)
        if landmarks is not None:
            # Visualize 2D landmarks
            self.mp_drawing.draw_landmarks(
                viz_image,
                results.pose_landmarks,
                mp.solutions.pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style()
            )
            
            # Calculate 3D positions and angles
            self.landmarks_3d = self.get_3d_landmarks(landmarks, self.depth_image)
            angles = self.get_pepper_angles(landmarks, self.depth_image)
            
            if angles is not None:
                # Visualize angles on image
                angle_y_pos = 30
                colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 255, 0), 
                          (0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 255, 0)]
                
                for idx, (joint, angle) in enumerate(angles.items()):
                    color = colors[idx % len(colors)]
                    cv2.putText(viz_image, f"{joint}: {angle:.2f} deg", 
                                (10, angle_y_pos), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.6, color, 2)
                    angle_y_pos += 25
                
                current_time = rospy.get_time()
                if angles is not None and (current_time - self.last_logged_time) >= 1.0:
                    rospy.loginfo("Joint Angles:")
                    colors = ["\033[91m", "\033[92m", "\033[93m", "\033[94m",
                            "\033[91m", "\033[92m", "\033[93m", "\033[94m"]
                    for idx, (joint, angle) in enumerate(angles.items()):
                        color = colors[idx % len(colors)]
                        rospy.loginfo(f"{color}{joint}: {angle:.2f} radians\033[0m")

                    self.last_logged_time = current_time

                # Update 3D visualization
                self.visualize_3d_skeleton()
        else:
            cv2.putText(viz_image, "No pose detected", (20, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            rospy.logwarn("Landmarks could not be calculated.")

        # Publish visualization
        try:
            self.visualization_pub.publish(self.bridge.cv2_to_imgmsg(viz_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error (visualization): {e}")

    def get_landmarks(self, bgr_image):
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        rgb_image.flags.writeable = False
        results = self.mp_pose.process(rgb_image)
        if results.pose_landmarks:
            return results.pose_landmarks.landmark, results
        else:
            return None, None
    
    def get_3d_landmarks(self, landmarks, depth_image):
        landmarks_3d = {}
        
        # Define the landmark indices we're interested in
        landmark_indices = {
            'LEFT_SHOULDER': mp.solutions.pose.PoseLandmark.LEFT_SHOULDER.value,
            'RIGHT_SHOULDER': mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER.value,
            'LEFT_ELBOW': mp.solutions.pose.PoseLandmark.LEFT_ELBOW.value,
            'RIGHT_ELBOW': mp.solutions.pose.PoseLandmark.RIGHT_ELBOW.value,
            'LEFT_WRIST': mp.solutions.pose.PoseLandmark.LEFT_WRIST.value,
            'RIGHT_WRIST': mp.solutions.pose.PoseLandmark.RIGHT_WRIST.value,
            'LEFT_HIP': mp.solutions.pose.PoseLandmark.LEFT_HIP.value,
            'RIGHT_HIP': mp.solutions.pose.PoseLandmark.RIGHT_HIP.value,
            'NOSE': mp.solutions.pose.PoseLandmark.NOSE.value,
        }
        
        for name, idx in landmark_indices.items():
            coord = self.get_3d_coordinates(landmarks[idx], depth_image)
            if coord is not None:
                landmarks_3d[name] = coord
        
        # Calculate mid points if we have the necessary landmarks
        if 'LEFT_SHOULDER' in landmarks_3d and 'RIGHT_SHOULDER' in landmarks_3d:
            landmarks_3d['NECK'] = np.array(mid_point(
                landmarks_3d['LEFT_SHOULDER'], landmarks_3d['RIGHT_SHOULDER']))
            
        if 'LEFT_HIP' in landmarks_3d and 'RIGHT_HIP' in landmarks_3d:
            landmarks_3d['MID_HIP'] = np.array(mid_point(
                landmarks_3d['LEFT_HIP'], landmarks_3d['RIGHT_HIP']))
        
        return landmarks_3d
        
    def checkLim(self, val, limits):
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
    
    def visualize_3d_skeleton(self):
        """
        Visualize the 3D skeleton. This method has two approaches:
        1. Interactive matplotlib figure (if matplotlib is available)
        2. Simple OpenCV-based 3D visualization (as fallback)
        """
        if len(self.landmarks_3d) < 5:
            return
            
        # Create a visual representation of the 3D skeleton using just OpenCV
        # This is more reliable than matplotlib conversion
        canvas_size = 500
        canvas = np.ones((canvas_size, canvas_size, 3), dtype=np.uint8) * 255
        
        # Extract coordinates for visualization
        landmarks_list = []
        for name, pos in self.landmarks_3d.items():
            landmarks_list.append((name, pos))
        
        if not landmarks_list:
            return
            
        # Find min and max values for normalization
        all_coords = np.array([pos for _, pos in landmarks_list])
        min_vals = np.min(all_coords, axis=0)
        max_vals = np.max(all_coords, axis=0)
        
        # Normalize to fit in the canvas with padding
        padding = 50
        scale = (canvas_size - 2 * padding) / max(max_vals - min_vals)
        
        # Function to convert 3D coords to 2D canvas coords
        def project_to_canvas(point_3d):
            # Simple orthographic projection (top view and side view)
            normalized = (point_3d - min_vals) * scale + padding
            # For top view (X-Y)
            top_x, top_y = int(normalized[0]), int(normalized[1])
            # For side view (X-Z)
            side_x, side_y = int(normalized[0]), canvas_size - int(normalized[2])
            return (top_x, top_y), (side_x, side_y)
        
        # Draw dividing line between views
        cv2.line(canvas, (0, canvas_size//2), (canvas_size, canvas_size//2), (0, 0, 0), 2)
        
        # Add labels for views
        cv2.putText(canvas, "Top View (X-Y)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        cv2.putText(canvas, "Side View (X-Z)", (10, canvas_size//2 + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        
        # Convert landmarks to pixel coordinates and draw them
        pixel_coords = {}
        for name, pos in landmarks_list:
            top_coords, side_coords = project_to_canvas(pos)
            
            # Store pixel coordinates
            pixel_coords[name] = (top_coords, side_coords)
            
            # Draw top view
            cv2.circle(canvas, top_coords, 5, (255, 0, 0), -1)
            cv2.putText(canvas, name, (top_coords[0] + 5, top_coords[1] + 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
            
            # Draw side view
            cv2.circle(canvas, side_coords, 5, (255, 0, 0), -1)
        
        # Connect points with lines to form skeleton
        connections = [
            ('NOSE', 'NECK'),
            ('NECK', 'LEFT_SHOULDER'),
            ('NECK', 'RIGHT_SHOULDER'),
            ('LEFT_SHOULDER', 'LEFT_ELBOW'),
            ('LEFT_ELBOW', 'LEFT_WRIST'),
            ('RIGHT_SHOULDER', 'RIGHT_ELBOW'),
            ('RIGHT_ELBOW', 'RIGHT_WRIST'),
            ('NECK', 'MID_HIP'),
            ('MID_HIP', 'LEFT_HIP'),
            ('MID_HIP', 'RIGHT_HIP')
        ]
        
        for start, end in connections:
            if start in pixel_coords and end in pixel_coords:
                # Draw line in top view
                start_top, _ = pixel_coords[start]
                end_top, _ = pixel_coords[end]
                cv2.line(canvas, start_top, end_top, (0, 255, 0), 2)
                
                # Draw line in side view
                _, start_side = pixel_coords[start]
                _, end_side = pixel_coords[end]
                cv2.line(canvas, start_side, end_side, (0, 255, 0), 2)
        
        # Also use matplotlib if available for a more sophisticated 3D view
        if self.fig3d is not None:
            try:
                # Clear previous plot
                self.ax3d.clear()
                
                # Set axis labels
                self.ax3d.set_xlabel('X')
                self.ax3d.set_ylabel('Y')
                self.ax3d.set_zlabel('Z')
                self.ax3d.set_title('3D Skeleton Visualization')
                
                # Plot points
                for name, pos in self.landmarks_3d.items():
                    self.ax3d.scatter(pos[0], pos[1], pos[2], marker='o', label=name)
                
                # Connect points with lines
                for start, end in connections:
                    if start in self.landmarks_3d and end in self.landmarks_3d:
                        x_vals = [self.landmarks_3d[start][0], self.landmarks_3d[end][0]]
                        y_vals = [self.landmarks_3d[start][1], self.landmarks_3d[end][1]]
                        z_vals = [self.landmarks_3d[start][2], self.landmarks_3d[end][2]]
                        self.ax3d.plot(x_vals, y_vals, z_vals, 'r-')
                
                # Set equal scale
                x_limits = self.ax3d.get_xlim3d()
                y_limits = self.ax3d.get_ylim3d()
                z_limits = self.ax3d.get_zlim3d()
                
                x_range = abs(x_limits[1] - x_limits[0])
                x_middle = np.mean(x_limits)
                y_range = abs(y_limits[1] - y_limits[0])
                y_middle = np.mean(y_limits)
                z_range = abs(z_limits[1] - z_limits[0])
                z_middle = np.mean(z_limits)
                
                max_range = 0.5 * max([x_range, y_range, z_range])
                
                self.ax3d.set_xlim3d([x_middle - max_range, x_middle + max_range])
                self.ax3d.set_ylim3d([y_middle - max_range, y_middle + max_range])
                self.ax3d.set_zlim3d([z_middle - max_range, z_middle + max_range])
                
                # Save figure to file instead of trying to convert directly
                self.fig3d.savefig('/tmp/skeleton_3d.png')
                self.fig3d.canvas.draw()
                self.fig3d.canvas.flush_events()
                
                # Try to read the saved image
                fig_img = cv2.imread('/tmp/skeleton_3d.png')
                if fig_img is not None:
                    try:
                        self.skeleton_3d_pub.publish(self.bridge.cv2_to_imgmsg(fig_img, "bgr8"))
                    except CvBridgeError as e:
                        rospy.logerr(f"CvBridge Error (3D matplotlib visualization): {e}")
                
            except Exception as e:
                rospy.logwarn(f"Matplotlib visualization failed: {e}")
        
        # Publish our OpenCV-based visualization
        try:
            self.skeleton_3d_pub.publish(self.bridge.cv2_to_imgmsg(canvas, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error (3D visualization): {e}")

if __name__ == "__main__":
    rospy.init_node('skeletal_model_estimation_node', anonymous=True)

    # intrinsics = [
    #     [606.12, 0.0, 321.38],
    #     [0.0, 605.72, 256.30],
    #     [0.0, 0.0, 1.0]
    # ]

    # K: [909.1842651367188, 0.0, 642.069580078125, 0.0, 908.5780639648438, 384.4430236816406, 0.0, 0.0, 1.0]

    intrinsics = [
        [909.1842651367188, 0.0, 642.069580078125],
        [0.0, 908.5780639648438, 384.4430236816406],
        [0.0, 0.0, 1.0]
    ]

    retargeting = HumanToPepperRetargeting()
    skeletal_estimator = SkeletalModelEstimationROS(
        camera_intrinsics=intrinsics,
        image_width=1280,
        image_height=720,
        retargeting=retargeting
    )

    rate = rospy.Rate(2)  # 2 Hz
    while not rospy.is_shutdown():
        skeletal_estimator.process_image()
        rate.sleep()