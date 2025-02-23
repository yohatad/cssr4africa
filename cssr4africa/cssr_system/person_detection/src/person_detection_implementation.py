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
                depth_array = np.array(self.depth_image, dtype=np.float32)
                depth_array = np.nan_to_num(depth_array, nan=0.0, posinf=0.0, neginf=0.0)
                normalized_depth = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX)
                normalized_depth = np.uint8(normalized_depth)
                depth_colormap = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)
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

        if x < 0 or x >= width or y < 0 or y >= height:
            rospy.logwarn(f"Centroid coordinates ({x}, {y}) are out of bounds.")
            return None

        depth_value = self.depth_image[y, x]
        if np.isfinite(depth_value) and depth_value > 0:
            return depth_value / 1000.0  # Convert if needed
        else:
            return None

    def publish_person_detection(self, tracking_data):
        """Publish the detected people to the topic."""
        # person_msg = person_detection()
        # person_msg.person_label_id = [data['track_id'] for data in tracking_data]
        # person_msg.centroid = [data['centroid'] for data in tracking_data]
        # self.pub_people.publish(person_msg)

class YOLOv8(PersonDetectionNode):
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

        # Generate a random color palette for drawing multiple people distinctly
        rng = np.random.default_rng(3)
        self.colors = rng.uniform(0, 255, size=(50, 3))  # 50 random colors
        
        self.latest_frame = None
        self.bridge = CvBridge()
        
        rospy.loginfo("Person detection node initialized")
        self.subscribe_topics()

    def _init_model(self):
        """Loads the ONNX model and prepares the runtime session."""
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
        """
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        boxes, scores, class_ids = self.detect_object(frame)

        # Update the latest frame only with bounding boxes if any remain
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
        outputs = self.session.run(
            [o.name for o in self.session.get_outputs()],
            {self.session.get_inputs()[0].name: model_input}
        )
        return self.process_output(outputs)

    def prepare_input(self, image):
        """Converts the image to RGB, resizes, normalizes, and transposes it for inference."""
        self.orig_height, self.orig_width = image.shape[:2]
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb_image, (self.input_width, self.input_height)).astype(np.float32)
        resized /= 255.0
        return resized.transpose(2, 0, 1)[None]

    def process_output(self, model_output):
        """
        Interprets the raw model output to filter boxes, scores, and classes 
        according to confidence threshold and NMS, then keeps only 'person' class.
        """
        preds = np.squeeze(model_output[0]).T  # shape: [num_boxes, :]
        conf_scores = np.max(preds[:, 4:], axis=1)
        mask = conf_scores > self.conf_threshold
        preds, conf_scores = preds[mask], conf_scores[mask]

        if not len(conf_scores):
            return np.array([]), np.array([]), np.array([])

        class_ids = np.argmax(preds[:, 4:], axis=1)
        boxes = preds[:, :4]
        boxes = self.rescale_boxes(boxes)
        boxes = self.xywh2xyxy(boxes)

        keep_idx = self.multiclass_nms(boxes, conf_scores, class_ids, self.iou_threshold)

        # Final results after NMS
        boxes, conf_scores, class_ids = boxes[keep_idx], conf_scores[keep_idx], class_ids[keep_idx]

        # ---- FILTER ONLY 'PERSON' (COCO class index 0) ----
        person_mask = (class_ids == 0)
        boxes = boxes[person_mask]
        conf_scores = conf_scores[person_mask]
        class_ids = class_ids[person_mask]

        return boxes, conf_scores, class_ids

    def rescale_boxes(self, boxes):
        """
        Converts boxes from model input scale (self.input_width/height)
        to the original image scale (self.orig_width/height).
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
        """Converts [x_center, y_center, w, h] to [x1, y1, x2, y2] in-place."""
        x, y, w, h = [boxes[:, i].copy() for i in range(4)]
        boxes[:, 0] = x - w / 2
        boxes[:, 1] = y - h / 2
        boxes[:, 2] = x + w / 2
        boxes[:, 3] = y + h / 2
        return boxes

    def multiclass_nms(self, boxes, scores, class_ids, iou_threshold):
        """Performs standard NMS on a per-class basis, collects kept indices."""
        final_keep = []
        for cid in np.unique(class_ids):
            idx = np.where(class_ids == cid)[0]
            keep = self.nms(boxes[idx], scores[idx], iou_threshold)
            final_keep.extend(idx[k] for k in keep)
        return final_keep

    def nms(self, boxes, scores, iou_threshold):
        """Single-class NMS."""
        sorted_idx = np.argsort(scores)[::-1]
        kept_indices = []
        while len(sorted_idx):
            curr = sorted_idx[0]
            kept_indices.append(curr)
            ious = self.compute_iou(boxes[curr], boxes[sorted_idx[1:]])
            sorted_idx = sorted_idx[1:][ious < iou_threshold]
        return kept_indices

    def compute_iou(self, main_box, all_boxes):
        """Computes IoU between one box and an array of boxes."""
        x1 = np.maximum(main_box[0], all_boxes[:, 0])
        y1 = np.maximum(main_box[1], all_boxes[:, 1])
        x2 = np.minimum(main_box[2], all_boxes[:, 2])
        y2 = np.minimum(main_box[3], all_boxes[:, 3])

        inter_w = np.maximum(0, x2 - x1)
        inter_h = np.maximum(0, y2 - y1)
        inter_area = inter_w * inter_h

        box_area = (main_box[2] - main_box[0]) * (main_box[3] - main_box[1])
        boxes_area = (all_boxes[:, 2] - all_boxes[:, 0]) * (all_boxes[:, 3] - all_boxes[:, 1])
        return inter_area / (box_area + boxes_area - inter_area)

    def draw_detections(self, image, boxes, scores, class_ids, mask_alpha=0.3):
        """
        Draw bounding boxes and labels on the image. Each detected person
        is assigned a distinct color from the 'colors' array.
        """
        output_img = image.copy()
        h, w = image.shape[:2]
        font_scale = 0.0006 * min(h, w)
        text_thickness = int(0.001 * min(h, w))

        # Optionally draw a mask overlay if you want semi-transparent bounding boxes
        # or just skip it if you want standard rectangles
        output_img = self.draw_masks(output_img, boxes, class_ids, mask_alpha)

        for i, (box, score) in enumerate(zip(boxes, scores)):
            # Use a different color for each detection
            color = self.colors[i % len(self.colors)]
            self.draw_box(output_img, box, color)
            label_text = f"person {int(score * 100)}%"
            self.draw_text(output_img, label_text, box, color, font_scale, text_thickness)

        return output_img

    def draw_masks(self, image, boxes, class_ids, alpha=0.3):
        """
        Draws semi-transparent colored rectangles for each detection.
        Since we're only detecting 'person', class_ids are all 0,
        but we use a unique color per detection for variety.
        """
        mask_img = image.copy()
        for i, box in enumerate(boxes):
            color = self.colors[i % len(self.colors)]
            x1, y1, x2, y2 = box.astype(int)
            cv2.rectangle(mask_img, (x1, y1), (x2, y2), color, -1)
        return cv2.addWeighted(mask_img, alpha, image, 1 - alpha, 0)

    def draw_box(self, image, box, color=(0, 0, 255), thickness=2):
        """Draw a rectangle for one bounding box."""
        x1, y1, x2, y2 = box.astype(int)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
        return image

    def draw_text(self, image, text_str, box, color=(0, 0, 255), font_scale=0.5, thickness=1):
        """Draw a text label above the bounding box."""
        x1, y1, x2, y2 = box.astype(int)
        (tw, th), _ = cv2.getTextSize(text_str, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        th = int(th * 1.2)
        cv2.rectangle(image, (x1, y1), (x1 + tw, y1 - th), color, -1)
        cv2.putText(image, text_str, (x1, y1),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
        return image

    def spin(self):
        """Main loop for ROS callbacks and display."""
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