#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import onnxruntime
from math import cos, sin, pi
from typing import Tuple, List
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import multiprocessing
import threading
from queue import Queue

class GoldYOLOONNX:
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

class GoldYOLONode:
    def __init__(self):
        rospy.init_node('gold_yolo_node')
        model_path = rospy.get_param('~model', '/root/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/face_detection/models/gold_yolo_n_head_post_0277_0.5071_1x3x480x640.onnx')
        sixdrepnet_model_path = rospy.get_param('~sixdrepnet_model', '/root/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/face_detection/models/sixdrepnet360_Nx3x224x224.onnx')
        self.image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/gold_yolo/output_image')

        self.model = GoldYOLOONNX(model_path=model_path)
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

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

        # Initialize image queue and processing thread
        self.image_queue = Queue()
        self.processing_thread = threading.Thread(target=self.process_images)
        self.processing_thread.daemon = True
        self.processing_thread.start()

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
                draw_axis(debug_image, yaw_deg, pitch_deg, roll_deg, tdx=cx, tdy=cy, size=100)

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
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA
                )
                cv2.putText(
                    debug_image, text, (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1, cv2.LINE_AA
                )
                y_offset += 30  # Move to the next line

        try:
            output_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            self.image_pub.publish(output_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return

        # Reduce display overhead by commenting out the display code
        cv2.imshow("GoldYOLO Output", debug_image)
        cv2.waitKey(1)

def main():
    GoldYOLONode()
    rospy.spin()

if __name__ == "__main__":
    main()
