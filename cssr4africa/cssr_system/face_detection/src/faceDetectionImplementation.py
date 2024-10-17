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
from face_detection.msg import faceDetection  # Replace with your actual package name
from typing import Tuple, List
from queue import Queue

# ROS Node that supports switching between MediaPipe and SixDrepNet
class FaceDetectionNode:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        self.pub_gaze = rospy.Publisher("/faceDetection/data", faceDetection, queue_size=10)

    def resolve_model_path(self, path):
        if path.startswith('package://'):
            path = path[len('package://'):]
            package_name, relative_path = path.split('/', 1)
            rospack = rospkg.RosPack()
            package_path = rospack.get_path(package_name)
            path = os.path.join(package_path, relative_path)
        return path

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
        
    def image_callback(self, data):
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
        gaze_msg = faceDetection()
        gaze_msg.centroids = centroids
        gaze_msg.mutualGaze = mutualGaze_list
        self.pub_gaze.publish(gaze_msg)

class YOLOONNX:
    def __init__(
        self,
        model_path: str = 'gold_yolo_n_head_post_0277_0.5071_1x3x480x640.onnx',
        class_score_th: float = 0.65,
        providers: List[str] = ['CUDAExecutionProvider', 'CPUExecutionProvider'],
    ):
        self.class_score_th = class_score_th
        session_option = onnxruntime.SessionOptions()
        session_option.log_severity_level = 3
        
        # Optimize ONNX Runtime session options
        session_option.intra_op_num_threads = multiprocessing.cpu_count()
        session_option.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.onnx_session = onnxruntime.InferenceSession(
            model_path, sess_options=session_option, providers=providers
        )
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
        model_path = self.resolve_model_path(model_path_param)
        sixdrepnet_model_path = self.resolve_model_path(sixdrepnet_model_path_param)

        self.model = YOLOONNX(model_path=model_path)
        self.sixdrepnet_session = onnxruntime.InferenceSession(
            sixdrepnet_model_path,
            sess_options=onnxruntime.SessionOptions(),
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
        )

        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        self.image_queue = Queue()
        threading.Thread(target=self.process_images, daemon=True).start()

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

    def draw_axis(img, yaw, pitch, roll, tdx=None, tdy=None, size=100):
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

    def process_frame(self, cv_image):
        # Avoid unnecessary deep copies
        debug_image = cv_image.copy()
        boxes, scores = self.model(debug_image)
        img_h, img_w = debug_image.shape[:2]

        if len(boxes) > 0:
            boxes = np.array(boxes)
            indices = np.argsort(boxes[:, 0])
            boxes = boxes[indices]
            scores = scores[indices]

            looking_results = []
            input_tensors = []
            centers = []
            boxes_scores = []

            for idx, (box, score) in enumerate(zip(boxes, scores)):
                x1, y1, x2, y2 = box
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                w, h = x2 - x1, y2 - y1
                ew, eh = w * 1.2, h * 1.2
                ex1, ex2 = int(cx - ew / 2), int(cx + ew / 2)
                ey1, ey2 = int(cy - eh / 2), int(cy + eh / 2)
                ex1, ex2 = max(ex1, 0), min(ex2, img_w)
                ey1, ey2 = max(ey1, 0), min(ey2, img_h)

                head_image = debug_image[ey1:ey2, ex1:ex2]
                # Optimize image preprocessing
                resized_image = cv2.resize(head_image, (224, 224))
                normalized_image = (resized_image[..., ::-1] / 255.0 - self.mean) / self.std
                input_tensor = normalized_image.transpose(2, 0, 1)[np.newaxis, ...].astype(np.float32)
                input_tensors.append(input_tensor)
                centers.append((cx, cy))
                boxes_scores.append((box, score))

            # Batch processing for head pose estimation
            input_batch = np.vstack(input_tensors)
            yaw_pitch_rolls = self.sixdrepnet_session.run(None, {'input': input_batch})[0]

            for idx, (yaw_pitch_roll, (cx, cy), (box, score)) in enumerate(zip(yaw_pitch_rolls, centers, boxes_scores)):
                yaw_deg, pitch_deg, roll_deg = yaw_pitch_roll
                self.draw_axis(debug_image, yaw_deg, pitch_deg, roll_deg, tdx=cx, tdy=cy, size=100)

                # Determine if the person is looking forward
                if abs(yaw_deg) < 15 and abs(pitch_deg) < 15:
                    looking = "Forward"
                else:
                    looking = "Not Forward"
                looking_results.append(looking)

                x1, y1, x2, y2 = box
                # Minimize drawing operations
                cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 1)
                cv2.putText(
                    debug_image, f'{score:.2f}', (x1, max(y1 - 5, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA
                )

            # Display looking results at the top-left corner
            y_offset = 30  # Starting y position for text
            for idx, looking in enumerate(looking_results):
                text = f'Face {idx + 1}: {looking}'
                cv2.putText(
                    debug_image, text, (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                
                cv2.putText(
                    debug_image, text, (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1, cv2.LINE_AA)
                y_offset += 30  # Move to the next line

        try:
            output_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            self.pub_gaze.publish(output_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

        # Reduce display overhead by commenting out the display code
        cv2.imshow("SixDrepNet Output", debug_image)
        cv2.waitKey(1)

# Main function to select between MediaPipe and SixDrepNet based on ROS parameter
if __name__ == '__main__':
    rospy.init_node('face_detection_node', anonymous=True)
    detection_method = rospy.get_param('~detection_method', 'SixDrepNet')  # Choose 'MediaPipe' or 'SixDrepNet'

    if detection_method == 'MediaPipe':
        mp_node = MediaPipeFaceNode()
    elif detection_method == 'SixDrepNet':
        yolo_node = SixDrepNet()
    else:
        rospy.logerr("Invalid detection method specified")
        rospy.signal_shutdown("Invalid detection method")
    
    rospy.spin()
    cv2.destroyAllWindows()
