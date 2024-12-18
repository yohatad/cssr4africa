""""
face_detection_implementation.py Implementation code for running the face and Head Pose detection algorithm

Author: Yohannes Tadesse Haile
Date: December 15, 2024
Version: v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import cv2
import mediapipe as mp
import numpy as np
import rospy
import rospkg
import os
import onnxruntime
import multiprocessing
import json
from math import cos, sin, pi
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from typing import Tuple, List
from face_detection.msg import face_detection
from face_detection_tracking import Sort, CentroidTracker

class FaceDetectionNode:
    def __init__(self, config=None):
        if config is not None:
            self.config = config
        else:
            self.config = self.read_json_file()
        
        self.algorithm = self.config.get("algorithm", "mediapipe")
        self.pub_gaze = rospy.Publisher("/faceDetection/data", face_detection, queue_size=10)
        self.bridge = CvBridge()
        self.depth_image = None  # Initialize depth_image

    def subscribe_topics(self):
        camera_type = self.config.get("camera")
        if camera_type == "realsense":
            self.rgb_topic = self.extract_topics("RealSenseCameraRGB")
            self.depth_topic = self.extract_topics("RealSenseCameraDepth")
            rospy.loginfo(f"Subscribed to {self.rgb_topic}")
            rospy.loginfo(f"Subscribed to {self.depth_topic}")
        elif camera_type == "pepper":
            self.rgb_topic = self.extract_topics("PepperFrontCamera")
            self.depth_topic = self.extract_topics("PepperDepthCamera")
            rospy.loginfo(f"Subscribed to {self.rgb_topic}")
            rospy.loginfo(f"Subscribed to {self.depth_topic}")
        else:
            rospy.logerr("subscribe_topics: Invalid camera type specified")
            rospy.signal_shutdown("subscribe_topics: Invalid camera type")
            return

        if not self.rgb_topic or not self.depth_topic:
            rospy.logerr("subscribe_topics: Camera topic not found.")
            rospy.signal_shutdown("subscribe_topics: Camera topic not found")
            return

        self.image_sub = rospy.Subscriber(self.rgb_topic, Image, self.image_callback)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback)

    # check if the depth camera and color camera have the same resolution.
    def check_camera_resolution(self, rgb_image, depth_image):
        rgb_h, rgb_w = rgb_image.shape[:2]
        depth_h, depth_w = depth_image.shape[:2]
        return rgb_h == depth_h and rgb_w == depth_w

    def resolve_model_path(self, path):
        if path.startswith('package://'):
            path = path[len('package://'):]
            package_name, relative_path = path.split('/', 1)
            rospack = rospkg.RosPack()
            package_path = rospack.get_path(package_name)
            path = os.path.join(package_path, relative_path)
        return path
    
    @staticmethod
    def read_json_file():
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('face_detection')
            config_path = os.path.join(package_path, 'config', 'face_detection_configuration.json')
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    data = json.load(file)
                    return data
            else:
                rospy.logerr(f"read_json_file: Configuration file not found at {config_path}")
        
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"ROS package 'face_detection' not found: {e}")
    
    @staticmethod
    def extract_topics(image_topic):
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('face_detection')
            config_path = os.path.join(package_path, 'data', 'pepper_topics.dat')

            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    for line in file:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            continue
                        key, value = line.split(maxsplit=1)
                        if key.lower() == image_topic.lower():
                            return value
            else:
                rospy.logerr(f"extract_topics: Data file not found at {config_path}")
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"ROS package 'face_detection' not found: {e}")
      
    def depth_callback(self, data):
        """Callback to receive the depth image."""
        try:
            # Convert the ROS Image message to a NumPy array
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr("depth_callback: CvBridge Error: {}".format(e))

    def display_depth_image(self):
        if self.depth_image is not None:
            try:
                # Convert depth image to float32 for processing
                depth_array = np.array(self.depth_image, dtype=np.float32)

                # Handle invalid depth values (e.g., NaNs, infs)
                depth_array = np.nan_to_num(depth_array, nan=0.0, posinf=0.0, neginf=0.0)

                # Normalize the depth image to the 0-255 range
                normalized_depth = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX)

                # Convert to 8-bit image
                normalized_depth = np.uint8(normalized_depth)

                # Apply a colormap for better visualization (optional)
                depth_colormap = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)

                # Display the depth image
                cv2.imshow("Depth Image", depth_colormap)

            except Exception as e:
                rospy.logerr("display_depth_image: Error displaying depth image: {}".format(e))

    def get_depth_at_centroid(self, centroid_x, centroid_y):
        """Get the depth value at the centroid of a face."""
        if self.depth_image is None:
            return None

        height, width = self.depth_image.shape[:2]
        x = int(round(centroid_x))
        y = int(round(centroid_y))

        # Check bounds
        if x < 0 or x >= width or y < 0 or y >= height:
            rospy.logwarn(f"Centroid coordinates ({x}, {y}) are out of bounds.")
            return None

        depth_value = self.depth_image[y, x]

        # Handle invalid depth values
        if np.isfinite(depth_value) and depth_value > 0:
            # Convert to meters if necessary
            depth_in_meters = depth_value / 1000.0
            return depth_in_meters
        else:
            # rospy.logwarn(f"Invalid depth value at coordinates ({x}, {y}): {depth_value}")
            return None

    def publish_face_detection(self, tracking_data):
        """Publish the face detection results."""
        face_msg = face_detection()

        # Initialize lists for each attribute in the message
        face_msg.face_label_id = [data['track_id'] for data in tracking_data]
        face_msg.centroids = [data['centroid'] for data in tracking_data]
        face_msg.mutualGaze = [data['mutual_gaze'] for data in tracking_data]

        # Publish the message
        self.pub_gaze.publish(face_msg)
class MediaPipeFaceNode(FaceDetectionNode):
    def __init__(self, config):
        super().__init__(config)
        # Initialize MediaPipe components
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(self.config.get("mediapipe_confidence", 0.5), max_num_faces=10)
        
        self.mp_face_detection = mp.solutions.face_detection
        self.face_detection = self.mp_face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5)
        
        self.mp_drawing = mp.solutions.drawing_utils
        self.drawing_spec = self.mp_drawing.DrawingSpec(color=(128, 128, 128), thickness=1, circle_radius=1)

        # Initialize the CentroidTracker
        self.centroid_tracker = CentroidTracker(self.config.get("max_disappeared", 15), self.config.get("distance_threshold", 100))
        
        self.latest_frame = None

        self.verbose_mode = bool(self.config.get("verbose_mode", False))

        # Timer for printing message every 5 seconds
        self.timer = rospy.get_time()
        
        # Subscribe to the image topic
        self.subscribe_topics()

        # check if the depth camera and color camera have the same resolution.
        if self.depth_image is not None:
            if not self.check_camera_resolution(self.latest_frame, self.depth_image):
                rospy.logerr("Color camera and depth camera have different resolutions.")
                rospy.signal_shutdown("Resolution mismatch")
        
    def image_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_h, img_w, _ = frame.shape

        # Process with face mesh
        self.process_face_mesh(frame, rgb_frame, img_h, img_w)

        # Print message every 5 seconds
        if rospy.get_time() - self.timer > 5:
            rospy.loginfo("face_detection: running.")
            self.timer = rospy.get_time()

        # Store the processed frame for dispay
        self.latest_frame = frame.copy()

    def spin(self):
        """Main loop to display processed frames and depth images."""
        rate = rospy.Rate(30)  # Adjust the rate as needed
        while not rospy.is_shutdown():
            if self.latest_frame is not None:
                if self.verbose_mode:
                    # Display the processed frame
                    cv2.imshow("Face Detection & Head Pose Estimation", self.latest_frame)

            # Display the depth image if verbose mode is enabled
            if self.verbose_mode:
                self.display_depth_image()

            # Wait for GUI events
            if cv2.waitKey(1) & 0xFF == ord("q"):
                rospy.signal_shutdown("User requested shutdown")

            rate.sleep()

        # Clean up OpenCV windows on shutdown
        cv2.destroyAllWindows()

    def process_face_mesh(self, frame, rgb_frame, img_h, img_w):
        results = self.face_mesh.process(rgb_frame)
        centroids = []
        mutualGaze_list = []

        if results.multi_face_landmarks:
            for face_id, face_landmarks in enumerate(results.multi_face_landmarks):
                face_2d = []
                face_3d = []

                for idx, lm in enumerate(face_landmarks.landmark):
                    if idx in [33, 263, 1, 61, 291, 199]:
                        if idx == 1:
                            nose_2d = (lm.x * img_w, lm.y * img_h)
                        x, y = int(lm.x * img_w), int(lm.y * img_h)
                        face_2d.append([x, y])
                        face_3d.append([x, y, lm.z])

                centroid_x = np.mean([pt[0] for pt in face_2d])
                centroid_y = np.mean([pt[1] for pt in face_2d])
                centroids.append((centroid_x, centroid_y))

                face_2d = np.array(face_2d, dtype=np.float64)
                face_3d = np.array(face_3d, dtype=np.float64)

                focal_length = 1 * img_w
                cam_matrix = np.array([[focal_length, 0, img_w / 2],
                                    [0, focal_length, img_h / 2],
                                    [0, 0, 1]])

                distortion_matrix = np.zeros((4, 1), dtype=np.float64)
                success, rotation_vec, translation_vec = cv2.solvePnP(
                    face_3d, face_2d, cam_matrix, distortion_matrix)

                rmat, _ = cv2.Rodrigues(rotation_vec)
                angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)
                x_angle = angles[0] * 360
                y_angle = angles[1] * 360

                mp_angle = self.config.get('mp_headpose_angle', 5)

                mutualGaze = abs(x_angle) <= mp_angle and abs(y_angle) <= mp_angle
                mutualGaze_list.append(mutualGaze)

                p1 = (int(nose_2d[0]), int(nose_2d[1]))
                p2 = (int(nose_2d[0] + y_angle * 10),
                    int(nose_2d[1] - x_angle * 10))
                cv2.line(frame, p1, p2, (255, 0, 0), 3)

                label = f"Face {face_id + 1}: {'Forward' if mutualGaze else 'Not Forward'}"
                cv2.putText(frame, label, (int(centroid_x), int(
                    centroid_y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Update the centroid tracker and get the object IDs
        tracked_faces = self.centroid_tracker.update(centroids)

        # Build tracking data list
        tracking_data = []

        # Create a mapping from tracked centroids to object IDs
        centroid_to_object_id = {}
        for object_id, tracked_centroid in tracked_faces.items():
            centroid_to_object_id[tuple(tracked_centroid)] = object_id

        # Match input centroids to tracked object IDs
        for idx, centroid in enumerate(centroids):
            centroid_tuple = tuple(centroid)
            # Find the closest tracked centroid to the input centroid
            min_distance = float('inf')
            matched_object_id = None
            for tracked_centroid_tuple, object_id in centroid_to_object_id.items():
                distance = np.linalg.norm(
                    np.array(centroid_tuple) - np.array(tracked_centroid_tuple))
                if distance < min_distance:
                    min_distance = distance
                    matched_object_id = object_id

            # Convert centroid to geometry_msgs/Point
            cz = self.get_depth_at_centroid(centroid[0], centroid[1])
            point = Point()
            point.x = float(centroid[0])
            point.y = float(centroid[1])
            point.z = float(cz) if cz else 0.0

            # Collect the tracking data with the correct data types
            tracking_data.append({
                'track_id': str(matched_object_id),             # Convert to string
                'centroid': point,                              # geometry_msgs/Point object
                'mutual_gaze': bool(mutualGaze_list[idx])       # Ensure it's a bool
            })

            # Annotate the frame with tracked face IDs
            cv2.putText(frame, f"ID {matched_object_id}", (int(centroid[0]), int(
                centroid[1]) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.circle(frame, (int(centroid[0]), int(centroid[1])),
                    4, (0, 255, 0), -1)
            
        # Publish centroids and mutual gaze status
        self.publish_face_detection(tracking_data)
class YOLOONNX:
    def __init__(self, model_path: str, class_score_th: float = 0.65,
        providers: List[str] = ['CUDAExecutionProvider', 'CPUExecutionProvider']):
        self.class_score_th = class_score_th
        session_option = onnxruntime.SessionOptions()
        session_option.log_severity_level = 3
        
        # Optimize ONNX Runtime session options
        session_option.intra_op_num_threads = multiprocessing.cpu_count()
        session_option.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.onnx_session = onnxruntime.InferenceSession(
            model_path, sess_options=session_option, providers=providers)
        
        self.input_shape = self.onnx_session.get_inputs()[0].shape
        self.input_names = [inp.name for inp in self.onnx_session.get_inputs()]
        self.output_names = [out.name for out in self.onnx_session.get_outputs()]

    def __call__(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        resized_image = self.__preprocess(image)
        inference_image = resized_image[np.newaxis, ...].astype(np.float32)
        boxes = self.onnx_session.run(
            self.output_names,
            {name: inference_image for name in self.input_names},
        )[0]
        return self.__postprocess(image, boxes)

    def __preprocess(self, image: np.ndarray) -> np.ndarray:
        resized_image = cv2.resize(image, (self.input_shape[3], self.input_shape[2]))
        resized_image = resized_image[:, :, ::-1] / 255.0  # BGR to RGB and normalize
        return resized_image.transpose(2, 0, 1)  # HWC to CHW

    def __postprocess(self, image: np.ndarray, boxes: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        img_h, img_w = image.shape[:2]
        result_boxes = []
        result_scores = []
        if boxes.size > 0:
            scores = boxes[:, 6]
            keep_idxs = scores > self.class_score_th
            boxes_keep = boxes[keep_idxs]
            for box in boxes_keep:
                x_min = int(max(box[2], 0) * img_w / self.input_shape[3])
                y_min = int(max(box[3], 0) * img_h / self.input_shape[2])
                x_max = int(min(box[4], self.input_shape[3]) * img_w / self.input_shape[3])
                y_max = int(min(box[5], self.input_shape[2]) * img_h / self.input_shape[2])
                result_boxes.append([x_min, y_min, x_max, y_max])
                result_scores.append(box[6])
        return np.array(result_boxes), np.array(result_scores)

class SixDrepNet(FaceDetectionNode):
    def __init__(self, config):
        super().__init__(config)
        self.initialized = False
        self.verbose_mode = self.config.get("verbose_mode", False)

        if self.verbose_mode:
            rospy.loginfo("Initializing SixDrepNet...")

        # Set up model paths
        model_path_param = 'package://face_detection/models/face_detection_goldYOLO.onnx'
        sixdrepnet_model_path_param = 'package://face_detection/models/face_detection_sixdrepnet360.onnx'
        
        yolo_model_path = self.resolve_model_path(model_path_param)
        sixdrepnet_model_path = self.resolve_model_path(sixdrepnet_model_path_param)

        self.latest_frame = None
        
        # Timer for printing message every 5 seconds
        self.timer = rospy.get_time()

        # Initialize YOLOONNX model early and check success
        try:
            self.yolo_model = YOLOONNX(model_path=yolo_model_path, class_score_th = self.config.get("sixdrepnet_confidence", 0.65))
            if self.verbose_mode:
                rospy.loginfo("YOLOONNX model initialized successfully.")
        except Exception as e:
            self.yolo_model = None
            rospy.logerr(f"Failed to initialize YOLOONNX model: {e}")
            return  # Exit early if initialization fails

        # Initialize SixDrepNet ONNX session
        try:
            session_option = onnxruntime.SessionOptions()
            session_option.log_severity_level = 3
            session_option.intra_op_num_threads = multiprocessing.cpu_count()
            session_option.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.sixdrepnet_session = onnxruntime.InferenceSession(
                sixdrepnet_model_path,
                sess_options=session_option,
                providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
            )

            active_providers = self.sixdrepnet_session.get_providers()
            if self.verbose_mode:
                rospy.loginfo(f"Active providers: {active_providers}")
            if "CUDAExecutionProvider" not in active_providers:
                if self.verbose_mode:
                    rospy.logwarn("CUDAExecutionProvider is not available. Running on CPU may slow down inference.")
            else:
                if self.verbose_mode:
                    rospy.loginfo("CUDAExecutionProvider is active. Running on GPU for faster inference.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize SixDrepNet ONNX session: {e}")
            return  # Exit early if initialization fails

        # Set up remaining attributes
        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

        self.sort_tracker = Sort(max_age=5, min_hits=3, iou_threshold=0.3)
        self.tracks = [] 
        
        # Mark initialization as complete
        self.initialized = True
        if self.verbose_mode:
            rospy.loginfo("SixDrepNet initialization complete.")

        self.subscribe_topics()
    
    def draw_axis(self, img, yaw, pitch, roll, tdx=None, tdy=None, size=100):
        pitch = pitch * pi / 180
        yaw = -yaw * pi / 180
        roll = roll * pi / 180
        height, width = img.shape[:2]
        tdx = tdx if tdx is not None else width / 2
        tdy = tdy if tdy is not None else height / 2

        x1 = size * (cos(yaw) * cos(roll)) + tdx
        y1 = size * (cos(pitch) * sin(roll) + sin(pitch) * sin(yaw) * cos(roll)) + tdy
        x2 = size * (-cos(yaw) * sin(roll)) + tdx
        y2 = size * (cos(pitch) * cos(roll) - sin(pitch) * sin(yaw) * sin(roll)) + tdy
        x3 = size * sin(yaw) + tdx
        y3 = size * (-cos(yaw) * sin(pitch)) + tdy

        cv2.line(img, (int(tdx), int(tdy)), (int(x1), int(y1)), (0, 0, 255), 2)
        cv2.line(img, (int(tdx), int(tdy)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.line(img, (int(tdx), int(tdy)), (int(x3), int(y3)), (255, 0, 0), 2)

    def image_callback(self, msg):
        if not self.initialized:
            rospy.logwarn("SixDrepNet is not fully initialized; skipping image callback.")
            return  # Skip processing if initialization is incomplete
             
        try:
            # Convert the ROS image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Process the frame
            self.latest_frame = self.process_frame(cv_image)

             # Print message every 5 seconds
            if rospy.get_time() - self.timer > 5:
                rospy.loginfo("face_detection: running.")
                self.timer = rospy.get_time()

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

    def process_frame(self, cv_image):
        
        """
        Process the input frame for face detection and head pose estimation using SORT.
        Args: 
            cv_image: Input frame as a NumPy array (BGR format)
        """
        debug_image = cv_image.copy()
        img_h, img_w = debug_image.shape[:2]
        tracking_data = []

        # Object detection (YOLO)
        boxes, scores = self.yolo_model(debug_image)

        # Prepare detections for SORT ([x1, y1, x2, y2, score])
        detections = []
        for box, score in zip(boxes, scores):
            x1, y1, x2, y2 = box
            detections.append([x1, y1, x2, y2, score])

        # Convert detections to NumPy array
        detections = np.array(detections)

        # Update SORT tracker with detections
        if detections.shape[0] > 0:
            self.tracks = self.sort_tracker.update(detections)
        else:
            self.tracks = []  # Reset tracks if no detections

        # Process tracks
        for track in self.tracks:
            x1, y1, x2, y2, track_id = map(int, track)  # SORT returns track_id as the last value
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            

            # Crop the face region for head pose estimation
            head_image = debug_image[max(y1, 0):min(y2, img_h), max(x1, 0):min(x2, img_w)]
            if head_image.size == 0:
                continue  # Skip if cropped region is invalid

            # Preprocess for SixDrepNet
            resized_image = cv2.resize(head_image, (224, 224))
            normalized_image = (resized_image[..., ::-1] / 255.0 - self.mean) / self.std
            input_tensor = normalized_image.transpose(2, 0, 1)[np.newaxis, ...].astype(np.float32)

            # Run head pose estimation
            yaw_pitch_roll = self.sixdrepnet_session.run(None, {'input': input_tensor})[0][0]
            yaw_deg, pitch_deg, roll_deg = yaw_pitch_roll

            # Draw head pose axes
            self.draw_axis(debug_image, yaw_deg, pitch_deg, roll_deg, cx, cy, size=100)

            cz = self.get_depth_at_centroid(cx, cy)

            # Track additional metadata
            sixdrep_angle = self.config.get('sixdrepnet_headpose_angle', 10)
            mutual_gaze = abs(yaw_deg) < sixdrep_angle and abs(pitch_deg) < sixdrep_angle
            tracking_data.append({
                'track_id': str(track_id),
                'centroid': Point(x=float(cx), y=float(cy), z=float(cz) if cz else 0.0),
                'mutual_gaze': mutual_gaze
            })

            # Draw bounding box and additional information
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 1)
            cv2.putText(debug_image, f"ID: {track_id}", (x1 + 10, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(debug_image, f"{'Forward' if mutual_gaze else 'Not Forward'}", (x1 + 10, y2 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Publish tracking data
        self.publish_face_detection(tracking_data)
        return debug_image
    
    def spin(self):
        """Main loop to display processed frames and depth images."""
        rate = rospy.Rate(30)  # Adjust the rate as needed
        while not rospy.is_shutdown():
            if self.latest_frame is not None:
                if self.verbose_mode:
                    # Display the processed frame
                    cv2.imshow("Face Detection & Head Pose Estimation", self.latest_frame)

            # Display the depth image if verbose mode is enabled
            if self.verbose_mode:
                self.display_depth_image()

            # Wait for GUI events
            if cv2.waitKey(1) & 0xFF == ord("q"):
                rospy.signal_shutdown("User requested shutdown")

            rate.sleep()

        # Clean up OpenCV windows on shutdown
        cv2.destroyAllWindows()