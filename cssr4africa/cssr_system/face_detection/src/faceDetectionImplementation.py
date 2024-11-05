#!/usr/bin/env python

""""
faceDetectionImplementation.py

Author: Yohannes Tadesse Haile
Date: November 1, 2024
Version: v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.

"""

"""
Description:
This file contains the implementation of the face detection using MediaPipe and SixDrepNet. The face detection
is implemented using the ROS image topic that could be configured to be the intel realsense camera or pepper robot
camera. It uses OpenCV to visualize the detected faces and gaze direction. The gaze direction is calculated using face
mesh landmarks which uses Google's MediaPipe library. The media pipe utilizes CPU for face detection and gaze direction.
The SixDrepNet uses YOLOONNX for face detection and SixDrepNet for gaze direction. The SixDrepNet utilizes GPU for faster
inference and better performance.

"""

import cv2
import mediapipe as mp
import numpy as np
import rospy
import rospkg
import os
import onnxruntime
import multiprocessing
from collections import OrderedDict
from scipy.spatial import distance as dist
from deep_sort_realtime.deepsort_tracker import DeepSort
from math import cos, sin, pi
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from face_detection.msg import faceDetection
from typing import Tuple, List

class FaceDetectionNode:
    def __init__(self):
        self.image_sub = None
        self.depth_sub = None
        self.config = self.parse_config()
        self.pub_gaze = rospy.Publisher("/faceDetection/data", faceDetection, queue_size=10)
        self.image_pub = rospy.Publisher("/faceDetection/image", Image, queue_size=10)
        self.bridge = CvBridge()

    def subscribe_topics(self):
        if self.config.get("camera") == "realsense":
            self.rgb_topic = self.extract_topics("realsensecamerargb")
            self.depth_topic = self.extract_topics("realsensecameradepth")
        elif self.config.get("camera") == "pepper":
            self.rgb_topic = self.extract_topics("PepperFrontCamera")
            self.depth_topic = None
        else:
            rospy.logerr("Invalid camera type specified")
            rospy.signal_shutdown("Invalid camera type")

        if self.rgb_topic:
            self.image_sub = rospy.Subscriber(self.rgb_topic, Image, self.image_callback)
        if self.depth_topic:
            self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback)
        
    def resolve_model_path(self, path):
        if path.startswith('package://'):
            path = path[len('package://'):]
            package_name, relative_path = path.split('/', 1)
            rospack = rospkg.RosPack()
            package_path = rospack.get_path(package_name)
            path = os.path.join(package_path, relative_path)
        return path
    
    def parse_config(self):
        config = {}
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('face_detection')
            config_path = os.path.join(package_path, 'config', 'faceDetectionConfiguration.ini')
            
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    for line in file:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            continue
                        
                        # Split the line by the first space
                        key, value = line.split(maxsplit=1)
                        key = key.lower()  # Ensure the key is lowercase
                        
                        # Convert the value appropriately
                        try:
                            if '.' in value:
                                value = float(value)
                            else:
                                value = int(value)
                        except ValueError:
                            # Convert boolean strings and leave other strings in lowercase
                            if value.lower() == "true":
                                value = True
                            elif value.lower() == "false":
                                value = False
                            else:
                                value = value.lower()  # Convert to lowercase for string values
                        
                        config[key] = value

                # Colorize output: green for keys, cyan for values
                for key, value in config.items():
                    print(f"\033[36m{key}\033[0m: \033[93m{value}\033[0m")
            else:
                print(f"\033[91mConfiguration file not found at {config_path}\033[0m")
        except rospkg.ResourceNotFound as e:
            print(f"\033[91mROS package 'face_detection' not found: {e}\033[0m")
        
        return config

    def extract_topics(self, image_topic):
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('face_detection')
            config_path = os.path.join(package_path, 'data', 'pepperTopics.dat')

            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    for line in file:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            continue
                        key, value = line.split(maxsplit=1)
                        if key.lower() == image_topic:
                            return value
            else:
                print(f"\033[91mData file not found at {config_path}\033[0m")
        except rospkg.ResourceNotFound as e:
            print(f"\033[91mROS package 'face_detection' not found: {e}\033[0m")
      
    def depth_callback(self, data):
        """Callback to receive the depth image."""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

            # try:
            #     cv2.imshow("Depth Image", self.depth_image)
            #     if cv2.waitKey(1) & 0xFF == ord("q"):
            #         rospy.signal_shutdown("User requested shutdown")
            # except CvBridgeError as e:
            #     rospy.logerr("CvBridge Error: {}".format(e))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

    def get_depth_at_centroid(self, centroid_x, centroid_y):
        """Get the depth value at the centroid of a face."""
        if self.depth_image is None:
            return None
        # Ensure the centroid values are integers (pixel indices)
        x = int(centroid_x)
        y = int(centroid_y)
        depth_value = self.depth_image[y, x]  # Depth value in millimeters
        return depth_value / 1000.0  # Convert to meters
    
    def publish_face_detection(self, tracking_data):
        """Publish the face detection results."""
        face_msg = faceDetection()

        # Initialize lists for each attribute in the message
        face_msg.face_label_id = [data['track_id'] for data in tracking_data]
        face_msg.centroids = [data['centroid'] for data in tracking_data]
        face_msg.mutualGaze = [data['mutual_gaze'] for data in tracking_data]

        # Publish the message
        self.pub_gaze.publish(face_msg)

# Centroid tracker class for tracking objects. 
class CentroidTracker:
    def __init__(self, max_disappeared=50, distance_threshold=50):
        # Adjustable parameters
        self.max_disappeared = max_disappeared
        self.distance_threshold = distance_threshold

        # Internal variables
        self.next_object_id = 0
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()

    def register(self, centroid):
        """Registers a new object with the next available ID and resets its disappearance count."""
        self.objects[self.next_object_id] = centroid
        self.disappeared[self.next_object_id] = 0
        self.next_object_id += 1

    def deregister(self, object_id):
        """Removes an object from tracking."""
        if object_id in self.objects:
            del self.objects[object_id]
            del self.disappeared[object_id]

    def update(self, centroids):
        """Updates the tracker with new centroids from the current frame."""
        # If no centroids are detected, increase disappearance count for existing objects
        if len(centroids) == 0:
            for object_id in list(self.disappeared.keys()):
                self.disappeared[object_id] += 1
                if self.disappeared[object_id] > self.max_disappeared:
                    self.deregister(object_id)
            return self.objects

        # Convert input centroids to numpy array for easy computation
        input_centroids = np.array(centroids)

        # Register each centroid if there are no tracked objects
        if len(self.objects) == 0:
            for i in range(0, len(input_centroids)):
                self.register(input_centroids[i])
        else:
            # List of tracked object IDs and their current centroids
            object_ids = list(self.objects.keys())
            object_centroids = list(self.objects.values())

            # Compute distances between existing objects and new centroids
            D = dist.cdist(np.array(object_centroids), input_centroids)

            # Sort rows and columns by closest distances
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]

            used_rows = set()
            used_cols = set()

            # Match each tracked object with the closest new centroid if within the distance threshold
            for (row, col) in zip(rows, cols):
                if row in used_rows or col in used_cols:
                    continue

                # Check if distance is within the threshold for matching
                if D[row, col] > self.distance_threshold:
                    continue

                # Update the centroid of the matched object
                object_id = object_ids[row]
                self.objects[object_id] = input_centroids[col]
                self.disappeared[object_id] = 0

                used_rows.add(row)
                used_cols.add(col)

            # Process unmatched rows and columns
            unused_rows = set(range(0, D.shape[0])).difference(used_rows)
            unused_cols = set(range(0, D.shape[1])).difference(used_cols)

            # Mark unmatched existing objects as disappeared
            for row in unused_rows:
                object_id = object_ids[row]
                self.disappeared[object_id] += 1
                if self.disappeared[object_id] > self.max_disappeared:
                    self.deregister(object_id)

            # Register unmatched new centroids as new objects
            for col in unused_cols:
                self.register(input_centroids[col])

        return self.objects

class MediaPipeFaceNode(FaceDetectionNode):
    def __init__(self):
        super().__init__()
        # Initialize MediaPipe components
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(max_num_faces=10, min_detection_confidence=0.5)
        
        self.mp_face_detection = mp.solutions.face_detection
        self.face_detection = self.mp_face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5)
        
        self.mp_drawing = mp.solutions.drawing_utils
        self.drawing_spec = self.mp_drawing.DrawingSpec(color=(128, 128, 128), thickness=1, circle_radius=1)

        # Initialize the CentroidTracker
        self.centroid_tracker = CentroidTracker(max_disappeared=15, distance_threshold=100)

        # Subscribe to the image topic
        self.subscribe_topics()
        
    def image_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_h, img_w, _ = frame.shape

        # Process with face mesh
        self.process_face_mesh(frame, rgb_frame, img_h, img_w)

        # Display the frame using OpenCV
        cv2.imshow("Face Detection & Mesh", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rospy.signal_shutdown("User requested shutdown")

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

                mutualGaze = abs(x_angle) <= 5 and abs(y_angle) <= 5
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
            point = Point()
            point.x = centroid[0]
            point.y = centroid[1]
            point.z = 0.0  # You can set this to an appropriate value if available

            # Collect the tracking data with the correct data types
            tracking_data.append({
                'track_id': str(matched_object_id),  # Convert to string
                'centroid': point,                   # geometry_msgs/Point object
                'mutual_gaze': bool(mutualGaze_list[idx])  # Ensure it's a bool
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
    def __init__(self):
        super().__init__()
        self.initialized = False
        rospy.loginfo("Initializing SixDrepNet...")

        # Set up model paths
        model_path_param = 'package://face_detection/models/gold_yolo_n_head_post_0277_0.5071_1x3x480x640.onnx'
        sixdrepnet_model_path_param = 'package://face_detection/models/sixdrepnet360_Nx3x224x224.onnx'
        
        yolo_model_path = self.resolve_model_path(model_path_param)
        sixdrepnet_model_path = self.resolve_model_path(sixdrepnet_model_path_param)

        # Initialize YOLOONNX model early and check success
        try:
            self.yolo_model = YOLOONNX(model_path=yolo_model_path)
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
            rospy.loginfo(f"Active providers: {active_providers}")
            if "CUDAExecutionProvider" not in active_providers:
                rospy.logwarn("CUDAExecutionProvider is not available. Running on CPU may slow down inference.")
            else:
                rospy.loginfo("CUDAExecutionProvider is active. Running on GPU for faster inference.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize SixDrepNet ONNX session: {e}")
            return  # Exit early if initialization fails

        # Set up remaining attributes
        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        self.tracker = DeepSort(max_age=15, n_init=5, max_iou_distance=0.7, embedder='mobilenet', 
                                half=False, bgr=True, embedder_gpu=True)
        
        self.frame_counter = 0
        self.tracks = [] 
        # Mark initialization as complete
        self.initialized = True
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
            processed_frame = self.process_frame(cv_image)
            # Display the processed frame
            try:
                cv2.imshow("SixDrepNet Output", processed_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    rospy.signal_shutdown("User requested shutdown")
            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error: {e}")

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

    def process_frame(self, cv_image):
        debug_image = cv_image.copy()

        # Timing for object detection (YOLO)
        boxes, scores = self.yolo_model(debug_image)

        img_h, img_w = debug_image.shape[:2]
        detections = []
        tracking_data = []  # Store (track_id, centroid, mutual_gaze) for each track

        # Timing for head pose estimation (SixDrepNet)
        if len(boxes) > 0:
            input_tensors, centers, boxes_scores = [], [], []
            for box, score in zip(boxes, scores):
                x1, y1, x2, y2 = box
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                w, h = x2 - x1, y2 - y1
                ew, eh = w * 1.2, h * 1.2
                ex1, ex2 = max(int(cx - ew / 2), 0), min(int(cx + ew / 2), img_w)
                ey1, ey2 = max(int(cy - eh / 2), 0), min(int(cy + eh / 2), img_h)

                head_image = debug_image[ey1:ey2, ex1:ex2]
                resized_image = cv2.resize(head_image, (224, 224))
                normalized_image = (resized_image[..., ::-1] / 255.0 - self.mean) / self.std
                input_tensor = normalized_image.transpose(2, 0, 1)[np.newaxis, ...].astype(np.float32)
                input_tensors.append(input_tensor)
                centers.append((cx, cy))
                boxes_scores.append((box, score))
                detections.append(([int(x1), int(y1), int(w), int(h)], score))
            
            if self.frame_counter % 4 == 0:
                self.tracks = self.tracker.update_tracks(detections, frame=debug_image)

            self.frame_counter += 1

            # Draw bounding boxes and tracking IDs
            for track in self.tracks:
                if not track.is_confirmed():
                    continue
                track_id = track.track_id
                bbox = track.to_ltrb()  # Get bounding box (left, top, right, bottom)

                # Find matching center and perform head pose estimation
                yaw_pitch_rolls = self.sixdrepnet_session.run(None, {'input': np.vstack(input_tensors)})[0]
                for (yaw_pitch_roll, (cx, cy), (box, score)) in zip(yaw_pitch_rolls, centers, boxes_scores):
                    yaw_deg, pitch_deg, roll_deg = yaw_pitch_roll
                    self.draw_axis(debug_image, yaw_deg, pitch_deg, roll_deg, cx, cy, size=100)

                    cz = self.get_depth_at_centroid(cx, cy)
                    mutual_gaze = abs(yaw_deg) < 10 and abs(pitch_deg) < 10

                    # Store (track_id, centroid, mutual_gaze)
                    tracking_data.append({
                    'track_id': track_id,
                    'centroid': Point(x=float(cx), y=float(cy), z=float(cz)),
                    'mutual_gaze': mutual_gaze})

                    # Draw bounding box and additional information
                    x1, y1, x2, y2 = box
                    cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 1)
                    cv2.putText(debug_image, f"ID: {track_id}", (int(bbox[0]) + 200, int(bbox[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(debug_image, f"{'Forward' if mutual_gaze else 'Not Forward'}", (20, 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 3)
                    cv2.putText(debug_image, f"{cz:.2f} m", (x1 + 10, max(y1 - 5, 10)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)

        self.publish_face_detection(tracking_data)
        return debug_image


# Main function to select between MediaPipe and SixDrepNet based on ROS parameter
if __name__ == '__main__':
    rospy.init_node('face_detection_node', anonymous=True)
    detection_method = rospy.get_param('~detection_method', 'MediaPipe')  # Choose 'MediaPipe' or 'SixDrepNet'

    # face_node = FaceDetectionNode()
    # face_node.subscribe_topics()

    if detection_method == 'MediaPipe':
        mp_node = MediaPipeFaceNode()
    elif detection_method == 'SixDrepNet':
        yolo_node = SixDrepNet()
    else:
        rospy.logerr("Invalid detection method specified")
        rospy.signal_shutdown("Invalid detection method")
    
    rospy.spin()
    cv2.destroyAllWindows()
