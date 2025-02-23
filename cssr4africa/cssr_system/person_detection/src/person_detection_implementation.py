import cv2
import rospy
import os
import rospkg
import numpy as np
import onnxruntime
import multiprocessing
import json
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from person_detection.msg import person_detection

class_names = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
               'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
               'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
               'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
               'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
               'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
               'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard',
               'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
               'scissors', 'teddy bear', 'hair drier', 'toothbrush']

# Create a list of colors for each class where each color is a tuple of 3 integer values
rng = np.random.default_rng(3)
colors = rng.uniform(0, 255, size=(len(class_names), 3))

class PersonDetectionNode:
    def __init__(self):
        self.pub_people = rospy.Publisher("/personDetection/data", person_detection, queue_size=10)
        self.bridge = CvBridge()
        self.depth_image = None
    
    def subscribe_topics(self):
        camera_type = rospy.get_param("/personDetection_config/camera", "realsense")
        if camera_type == "realsense":
            self.rgb_topic = self.extract_topics("RealSenseCameraRGB")
            self.depth_topic = self.extract_topics("RealSenseCameraDepth")
        elif camera_type == "pepper":
            self.rgb_topic = self.extract_topics("PepperFrontCamera")
            self.depth_topic = self.extract_topics("PepperDepthCamera")
        else:
            rospy.logerr("Invalid camera type specified")
            rospy.signal_shutdown("Invalid camera type specified")

        self.image_sub = rospy.Subscriber(self.rgb_topic, Image, self.image_callback)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback)
        
    @staticmethod
    def read_json_file():
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('person_detection')
            config_path = os.path.join(package_path, 'config', 'person_detection_configuration.json')
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    data = json.load(file)
                    return data
            else:
                rospy.logerr(f"read_json_file: Configuration file not found at {config_path}")
        
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"ROS package 'person_detection not found: {e}") 

    @staticmethod
    def extract_topics(image_topic):
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('person_detection')
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
            rospy.logerr(f"ROS package 'person_detection' not found: {e}")

    def depth_callback(self, data):
        """Callback to receive the depth image."""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error in depth_callback: {}".format(e))
            return

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
                rospy.logerr("Error displaying depth image: {}".format(e))

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

    def publish_person_detection(self, tracking_data):
        """Publish the detected people to the topic."""
        person_msg = person_detection()

        # Initialize lists for each attribute in the message
        person_msg.person_label_id = [data['track_id'] for data in tracking_data]
        person_msg.centroid = [data['centroid'] for data in tracking_data]

        self.pub_people.publish(person_msg)

class YOLOv8ROS(PersonDetectionNode):
    def __init__(self):
        """
        Initializes the ROS node, loads configuration, and subscribes to necessary topics.
        """
        super().__init__()
        if not self._init_model():
            return
        
        # Load config parameters once
        config_params = rospy.get_param("/personDetection_config", {
            "verboseMode": False,
            "confidence_threshold": 0.5,
            "iou_threshold": 0.5
        })
        self.verbose_mode = config_params["verboseMode"]
        self.conf_threshold = config_params["confidence_threshold"]
        self.iou_threshold = config_params["iou_threshold"]
        
        self.latest_frame = None
        self.bridge = CvBridge()
        
        rospy.loginfo("Person detection node initialized")
        self.subscribe_topics()  # Ensure you have this method implemented or override if needed.

    def _init_model(self):
        """
        Loads the ONNX model and prepares the runtime session.
        
        Returns:
            bool: True if model is loaded successfully, otherwise False.
        """
        try:
            session_options = onnxruntime.SessionOptions()
            session_options.intra_op_num_threads = multiprocessing.cpu_count()
            session_options.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL

            model_path = rospkg.RosPack().get_path('person_detection') + '/models/yolov8s.onnx'
            self.session = onnxruntime.InferenceSession(
                model_path,
                sess_options=session_options,
                providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
            )

            input_shape = self.session.get_inputs()[0].shape  # [N, C, H, W]
            self.input_height, self.input_width = input_shape[2], input_shape[3]

            rospy.loginfo(f"ONNX model loaded. Providers: {self.session.get_providers()}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to initialize ONNX: {e}")
            return False

    def image_callback(self, msg):
        """
        Receives image messages from a ROS topic, runs detection,
        and stores the annotated frame for display in spin().
        
        Args:
            msg (sensor_msgs.msg.Image): The incoming image from a ROS topic.
        """
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        boxes, scores, class_ids = self.detect_object(frame)
        if len(boxes):
            self.latest_frame = self.draw_detections(frame, boxes, scores, class_ids)
        else:
            self.latest_frame = frame

    def detect_object(self, image):
        """
        Prepares the image and runs inference on the ONNX model.
        
        Args:
            image (np.ndarray): BGR image from OpenCV.
        
        Returns:
            tuple: (boxes, scores, class_ids)
        """
        model_input = self.prepare_input(image)
        model_output = self.session.run(
            [o.name for o in self.session.get_outputs()],
            {self.session.get_inputs()[0].name: model_input}
        )
        return self.process_output(model_output)

    def prepare_input(self, image):
        """
        Converts the image to RGB, resizes, normalizes, and transposes it for inference.

        Args:
            image (np.ndarray): Original BGR image.

        Returns:
            np.ndarray: 4D array of shape [1, 3, H, W].
        """
        self.orig_height, self.orig_width = image.shape[:2]
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb_image, (self.input_width, self.input_height)).astype(np.float32)
        resized /= 255.0
        return resized.transpose(2, 0, 1)[None]

    def process_output(self, model_output):
        """
        Interprets the raw model output to filter boxes, scores, and classes 
        according to confidence threshold and NMS.
        
        Args:
            model_output (list[np.ndarray]): The raw output from the ONNX model.
        
        Returns:
            tuple: (boxes, scores, class_ids) after confidence filtering and NMS.
        """
        preds = np.squeeze(model_output[0]).T  # shape: [num_boxes, :]
        scores = np.max(preds[:, 4:], axis=1)
        mask = scores > self.conf_threshold
        preds, scores = preds[mask], scores[mask]
        if not len(scores):
            return np.array([]), np.array([]), np.array([])

        class_ids = np.argmax(preds[:, 4:], axis=1)
        boxes = preds[:, :4]
        boxes = self.rescale_boxes(boxes)
        boxes = self.xywh2xyxy(boxes)

        keep_indices = self.multiclass_nms(boxes, scores, class_ids, self.iou_threshold)
        return boxes[keep_indices], scores[keep_indices], class_ids[keep_indices]

    def rescale_boxes(self, boxes):
        """
        Converts boxes from model input scale to original image scale.
        
        Args:
            boxes (np.ndarray): Boxes in [x_center, y_center, w, h] format.
        
        Returns:
            np.ndarray: Rescaled boxes in same format [x_center, y_center, w, h].
        """
        scale = np.array([
            self.orig_width / self.input_width,
            self.orig_height / self.input_height,
            self.orig_width / self.input_width,
            self.orig_height / self.input_height
        ], dtype=np.float32)
        boxes *= scale
        return boxes

    def xywh2xyxy(self, boxes):
        """
        Converts [x_center, y_center, w, h] to [x1, y1, x2, y2] in-place.
        
        Args:
            boxes (np.ndarray): Nx4 array of boxes in xywh format.
        
        Returns:
            np.ndarray: Nx4 array of boxes in xyxy format.
        """
        x, y, w, h = [boxes[:, i].copy() for i in range(4)]
        boxes[:, 0] = x - w / 2
        boxes[:, 1] = y - h / 2
        boxes[:, 2] = x + w / 2
        boxes[:, 3] = y + h / 2
        return boxes

    def multiclass_nms(self, boxes, scores, class_ids, iou_threshold):
        """
        Performs NMS separately for each class and collects kept indices.
        
        Args:
            boxes (np.ndarray): Nx4 array in [x1, y1, x2, y2].
            scores (np.ndarray): Confidence scores for each box.
            class_ids (np.ndarray): Class indices for each box.
            iou_threshold (float): IoU threshold for suppressing overlapping boxes.
        
        Returns:
            list: Indices of boxes that survive NMS.
        """
        final_keep = []
        for class_id in np.unique(class_ids):
            idx = np.where(class_ids == class_id)[0]
            keep = self.nms(boxes[idx], scores[idx], iou_threshold)
            final_keep.extend(idx[k] for k in keep)
        return final_keep

    def nms(self, boxes, scores, iou_threshold):
        """
        Performs standard single-class NMS.
        
        Args:
            boxes (np.ndarray): Nx4 in [x1, y1, x2, y2].
            scores (np.ndarray): 1D array of box confidence scores.
            iou_threshold (float): IoU threshold for NMS.
        
        Returns:
            list: Indices of boxes kept after suppression.
        """
        sorted_idx = np.argsort(scores)[::-1]
        kept_indices = []
        while len(sorted_idx):
            current_idx = sorted_idx[0]
            kept_indices.append(current_idx)
            ious = self.compute_iou(boxes[current_idx], boxes[sorted_idx[1:]])
            sorted_idx = sorted_idx[1:][ious < iou_threshold]
        return kept_indices

    def compute_iou(self, main_box, all_boxes):
        """
        Computes IoU between one box and an array of boxes.
        
        Args:
            main_box (np.ndarray): [x1, y1, x2, y2].
            all_boxes (np.ndarray): Nx4 array.
        
        Returns:
            np.ndarray: IoU for each of the boxes in all_boxes.
        """
        x1 = np.maximum(main_box[0], all_boxes[:, 0])
        y1 = np.maximum(main_box[1], all_boxes[:, 1])
        x2 = np.minimum(main_box[2], all_boxes[:, 2])
        y2 = np.minimum(main_box[3], all_boxes[:, 3])

        inter_w = np.maximum(0, x2 - x1)
        inter_h = np.maximum(0, y2 - y1)
        inter = inter_w * inter_h

        box_area = (main_box[2] - main_box[0]) * (main_box[3] - main_box[1])
        boxes_area = (all_boxes[:, 2] - all_boxes[:, 0]) * (all_boxes[:, 3] - all_boxes[:, 1])
        return inter / (box_area + boxes_area - inter)

    def draw_detections(self, image, boxes, scores, class_ids, mask_alpha=0.3):
        """
        Renders bounding boxes and labels on the image.
        
        Args:
            image (np.ndarray): Original BGR image.
            boxes (np.ndarray): Nx4 bounding boxes in xyxy format.
            scores (np.ndarray): Confidence scores.
            class_ids (np.ndarray): Class indices for each detection.
            mask_alpha (float): Transparency factor for the mask overlay.
        
        Returns:
            np.ndarray: Image with drawn boxes, masks, and labels.
        """
        output_image = image.copy()
        height, width = image.shape[:2]
        font_scale = 0.0006 * min(height, width)
        text_thickness = int(0.001 * min(height, width))

        output_image = self.draw_masks(output_image, boxes, class_ids, mask_alpha)
        for cid, box, score in zip(class_ids, boxes, scores):
            color = colors[cid]
            self.draw_box(output_image, box, color)
            label_text = f"{class_names[cid]} {int(score*100)}%"
            self.draw_text(output_image, label_text, box, color, font_scale, text_thickness)
        return output_image

    def draw_masks(self, image, boxes, class_ids, alpha=0.3):
        """
        Draws semi-transparent colored rectangles over detections.
        
        Args:
            image (np.ndarray): Original image to draw onto.
            boxes (np.ndarray): Nx4 bounding boxes in xyxy.
            class_ids (np.ndarray): Class indices.
            alpha (float): Transparency factor for overlay.
        
        Returns:
            np.ndarray: Image with a semi-transparent mask over bounding boxes.
        """
        mask_image = image.copy()
        for box, cid in zip(boxes, class_ids):
            x1, y1, x2, y2 = box.astype(int)
            cv2.rectangle(mask_image, (x1, y1), (x2, y2), colors[cid], -1)
        return cv2.addWeighted(mask_image, alpha, image, 1 - alpha, 0)

    def draw_box(self, image, box, color=(0, 0, 255), thickness=2):
        """
        Draws a rectangle on the image representing a bounding box.
        
        Args:
            image (np.ndarray): Original image.
            box (np.ndarray): [x1, y1, x2, y2].
            color (tuple): BGR color for the rectangle.
            thickness (int): Rectangle border thickness.
        
        Returns:
            np.ndarray: Image with a drawn rectangle.
        """
        x1, y1, x2, y2 = box.astype(int)
        return cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)

    def draw_text(self, image, text_str, box, color=(0, 0, 255), font_scale=0.5, thickness=1):
        """
        Draws text on top of the bounding box.
        
        Args:
            image (np.ndarray): Image to draw the text on.
            text_str (str): Text label.
            box (np.ndarray): [x1, y1, x2, y2].
            color (tuple): Color for the text background.
            font_scale (float): Font scale.
            thickness (int): Font thickness.
        
        Returns:
            np.ndarray: Image with the text label drawn.
        """
        x1, y1, _, _ = box.astype(int)
        (text_w, text_h), _ = cv2.getTextSize(text_str, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        text_h = int(text_h * 1.2)
        cv2.rectangle(image, (x1, y1), (x1 + text_w, y1 - text_h), color, -1)
        return cv2.putText(image, text_str, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, font_scale,
                           (255, 255, 255), thickness, cv2.LINE_AA)

    def spin(self):
        """
        Main loop to display the processed frames and optionally a depth image.
        """
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.latest_frame is not None:
                cv2.imshow("YOLOv8 ROS", self.latest_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.signal_shutdown("User requested shutdown")

            if self.verbose_mode:
                self.display_depth_image()

            rate.sleep()
        cv2.destroyAllWindows()