#!/usr/bin/env python

import cv2
import mediapipe as mp
import numpy as np
import rospy
import rospkg
import os
import onnxruntime
import multiprocessing
import threading
from math import cos, sin, pi
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from face_detection.msg import faceDetection
from typing import Tuple, List
from queue import Queue

class FaceDetectionNode:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.bridge = CvBridge()
        self.pub_gaze = rospy.Publisher("/faceDetection/data", faceDetection, queue_size=10)
        self.image_queue = Queue()
        self.multi_tracker = cv2.TrackerCSRT_create()

    def resolve_model_path(self, path):
        if path.startswith('package://'):
            path = path[len('package://'):]
            package_name, relative_path = path.split('/', 1)
            rospack = rospkg.RosPack()
            package_path = rospack.get_path(package_name)
            path = os.path.join(package_path, relative_path)
        return path
    
    def depth_callback(self, data):
        """Callback to receive the depth image."""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
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
    
    def publish_face_detection(self, centroids, mutual_gaze_list):
        """Publish the face detection results."""
        face_msg = faceDetection()
        # face_msg.face_label_id= face_label_id
        face_msg.centroids = centroids
        face_msg.mutualGaze = mutual_gaze_list
        self.pub_gaze.publish(face_msg)
    
class MediaPipeFaceNode(FaceDetectionNode):
    def __init__(self):
        super().__init__()
        # Initialize MediaPipe components
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(max_num_faces=10, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        
        self.mp_face_detection = mp.solutions.face_detection
        self.face_detection = self.mp_face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5)
        
        self.mp_drawing = mp.solutions.drawing_utils
        self.drawing_spec = self.mp_drawing.DrawingSpec(color=(128, 128, 128), thickness=1, circle_radius=1)
        
        rospy.loginfo("Successfully initialized MediaPipe components")
     
    def image_callback(self, data):
        # Check if face_detection is initialized
        if not hasattr(self, 'face_detection') or self.face_detection is None:
            return  # Skip this callback if initialization isn't complete
        
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_h, img_w, _ = frame.shape

        # Process with face detection
        self.process_face_detection(frame, rgb_frame, img_h, img_w)
        
        # Process with face mesh
        self.process_face_mesh(frame, rgb_frame, img_h, img_w)

        # Display the frame using OpenCV
        cv2.imshow("Face Detection & Mesh", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rospy.signal_shutdown("User requested shutdown")

    def process_face_detection(self, frame, rgb_frame, img_h, img_w):
        results = self.face_detection.process(rgb_frame)
        if results.detections:
            for face in results.detections:
                face_rect = np.multiply(
                    [
                        face.location_data.relative_bounding_box.xmin,
                        face.location_data.relative_bounding_box.ymin,
                        face.location_data.relative_bounding_box.width,
                        face.location_data.relative_bounding_box.height,
                    ],
                    [img_w, img_h, img_w, img_h]
                ).astype(int)
                # Draw bounding box
                cv2.rectangle(frame, face_rect, color=(255, 255, 255), thickness=2)

                # Extract and draw key points
                key_points = np.array([(p.x, p.y) for p in face.location_data.relative_keypoints])
                key_points_coords = np.multiply(key_points, [img_w, img_h]).astype(int)
                for p in key_points_coords:
                    cv2.circle(frame, tuple(p), 4, (255, 255, 255), 2)
                    cv2.circle(frame, tuple(p), 2, (0, 0, 0), -1)

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
                centroid = Point(x=centroid_x, y=centroid_y, z=0)
                centroids.append(centroid)

                face_2d = np.array(face_2d, dtype=np.float64)
                face_3d = np.array(face_3d, dtype=np.float64)

                focal_length = 1 * img_w
                cam_matrix = np.array([[focal_length, 0, img_h / 2],
                                       [0, focal_length, img_w / 2],
                                       [0, 0, 1]])

                distortion_matrix = np.zeros((4, 1), dtype=np.float64)
                success, rotation_vec, translation_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, distortion_matrix)

                rmat, _ = cv2.Rodrigues(rotation_vec)
                angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)
                x_angle = angles[0] * 360
                y_angle = angles[1] * 360

                mutualGaze = abs(x_angle) <= 5 and abs(y_angle) <= 5
                mutualGaze_list.append(mutualGaze)

                p1 = (int(nose_2d[0]), int(nose_2d[1]))
                p2 = (int(nose_2d[0] + y_angle * 10), int(nose_2d[1] - x_angle * 10))
                cv2.line(frame, p1, p2, (255, 0, 0), 3)

                label = f"Face {face_id + 1}: {'Forward' if mutualGaze else 'Not Forward'}"
                cv2.putText(frame, label, (int(centroid_x), int(centroid_y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                self.mp_drawing.draw_landmarks(
                    image=frame,
                    landmark_list=face_landmarks,
                    connections=self.mp_face_mesh.FACEMESH_CONTOURS,
                    landmark_drawing_spec=self.drawing_spec,
                    connection_drawing_spec=self.drawing_spec
                )

        # Publish centroids and mutual gaze status
        faceMsg = faceDetection()
        faceMsg.centroids = centroids
        faceMsg.mutualGaze = mutualGaze_list
        self.pub_gaze.publish(faceMsg)

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
        model_path_param = 'package://face_detection/models/gold_yolo_n_head_post_0277_0.5071_1x3x480x640.onnx'
        sixdrepnet_model_path_param = 'package://face_detection/models/sixdrepnet360_Nx3x224x224.onnx'
        
        # Resolve the package paths
        yolo_model_path = self.resolve_model_path(model_path_param)
        sixdrepnet_model_path = self.resolve_model_path(sixdrepnet_model_path_param)

        self.model = YOLOONNX(model_path=yolo_model_path)
        session_option = onnxruntime.SessionOptions()
        session_option.log_severity_level = 3

         # Optimize ONNX Runtime session options
        session_option.intra_op_num_threads = multiprocessing.cpu_count()
        session_option.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.sixdrepnet_session = onnxruntime.InferenceSession(
            sixdrepnet_model_path,
            sess_options=session_option,
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
        )

        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
           
        threading.Thread(target=self.process_images, daemon=True).start()
    
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
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_queue.put(cv_image)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

    def process_images(self):
        while not rospy.is_shutdown():
            if not self.image_queue.empty():
                cv_image = self.image_queue.get()
                self.process_frame(cv_image)

    def process_frame(self, cv_image):
        debug_image = cv_image.copy()
        boxes, scores = self.model(debug_image)
        img_h, img_w = debug_image.shape[:2]

        centroids, mutual_gaze_list, face_id_list = [], [], []

        if len(boxes) > 0:
            boxes = np.array(boxes)[np.argsort(np.array(boxes)[:, 0])]  # Sort boxes by x-coordinate
            scores = np.array(scores)[np.argsort(np.array(boxes)[:, 0])]  # Sort scores accordingly

            input_tensors, centers, boxes_scores = [], [], []

            for box, score in zip(boxes, scores):
                x1, y1, x2, y2 = box
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                w, h = x2 - x1, y2 - y1
                ew, eh = w * 1.2, h * 1.2
                ex1, ex2 = max(int(cx - ew / 2), 0), min(int(cx + ew / 2), img_w)
                ey1, ey2 = max(int(cy - eh / 2), 0), min(int(cy + eh / 2), img_h)

                # Preprocess image
                head_image = debug_image[ey1:ey2, ex1:ex2]
                resized_image = cv2.resize(head_image, (224, 224))
                normalized_image = (resized_image[..., ::-1] / 255.0 - self.mean) / self.std
                input_tensor = normalized_image.transpose(2, 0, 1)[np.newaxis, ...].astype(np.float32)
                input_tensors.append(input_tensor)
                centers.append((cx, cy))
                boxes_scores.append((box, score))

            # Batch process for head pose estimation
            yaw_pitch_rolls = self.sixdrepnet_session.run(None, {'input': np.vstack(input_tensors)})[0]

            for (yaw_pitch_roll, (cx, cy), (box, score)) in zip(yaw_pitch_rolls, centers, boxes_scores):
                yaw_deg, pitch_deg, roll_deg = yaw_pitch_roll
                self.draw_axis(debug_image, yaw_deg, pitch_deg, roll_deg, cx, cy, size=100)

                # Determine the depth at the centroid
                cz = self.get_depth_at_centroid(cx, cy)

                # Determine mutual gaze
                mutual_gaze = abs(yaw_deg) < 10 and abs(pitch_deg) < 10
                mutual_gaze_list.append(mutual_gaze)
                centroids.append(Point(x=float(cx), y=float(cy), z=float(cz)))

                # Minimize drawing operations
                x1, y1, x2, y2 = box
                cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 1)
                cv2.putText(debug_image, f'{score:.2f}', (x1, max(y1 - 5, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                for idx, mutual_gaze in enumerate(mutual_gaze_list):
                    text = f'Face {idx + 1}: {"Forward" if mutual_gaze else "Not Forward"}'
                    cv2.putText(debug_image, text, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)

            # Create and publish the faceMsg
            self.publish_face_detection(face_id_list, centroids, mutual_gaze_list)

        try:
            # Display output (if necessary)
            cv2.imshow("SixDrepNet Output", debug_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

# Main function to select between MediaPipe and SixDrepNet based on ROS parameter
if __name__ == '__main__':
    rospy.init_node('face_detection_node', anonymous=True)
    detection_method = rospy.get_param('~detection_method', 'MediaPipe')  # Choose 'MediaPipe' or 'SixDrepNet'

    if detection_method == 'MediaPipe':
        mp_node = MediaPipeFaceNode()
    elif detection_method == 'SixDrepNet':
        yolo_node = SixDrepNet()
    else:
        rospy.logerr("Invalid detection method specified")
        rospy.signal_shutdown("Invalid detection method")
    
    rospy.spin()
    cv2.destroyAllWindows()
