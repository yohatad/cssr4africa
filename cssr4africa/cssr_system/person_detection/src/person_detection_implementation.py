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
from person_detection_tracking import Sort

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

        # Generate a random color palette for bounding box drawing
        rng = np.random.default_rng(3)
        self.colors = rng.uniform(0, 255, size=(50, 3))  # 50 random colors

        # Instantiate SORT tracker (tuned for your scenario)
        self.tracker = Sort(max_age=5, min_hits=3, iou_threshold=0.3)

        self.latest_frame = None
        self.bridge = CvBridge()
        
        rospy.loginfo("Detection + Tracking node initialized")
        self.subscribe_topics()

    def _init_model(self):
        """Loads the ONNX model and prepares the runtime session."""
        try:
            so = onnxruntime.SessionOptions()
            so.intra_op_num_threads = multiprocessing.cpu_count()
            so.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL

            model_path = rospkg.RosPack().get_path('person_detection') + '/models/yolov8s.onnx'
            self.session = onnxruntime.InferenceSession(
                model_path,
                sess_options=so,
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
        Receives image messages from a ROS topic, runs detection, performs tracking,
        and stores the annotated frame for display in spin().
        """
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        boxes, scores, class_ids = self.detect_object(frame)

        # If no boxes, show original frame
        if not len(boxes):
            self.latest_frame = frame
            return

        # Convert YOLO output to Nx5: [x1, y1, x2, y2, score]
        # for input to the SORT tracker
        detections = np.hstack([boxes, scores.reshape(-1, 1)])  # shape: Nx5

        # --- Update the tracker ---
        tracked_objects = self.tracker.update(detections)  
        # tracked_objects is Nx5: [x1, y1, x2, y2, track_id]

        # Draw the results
        frame_with_tracks = self.draw_tracked_objects(frame, tracked_objects)
        self.latest_frame = frame_with_tracks

        # If you need to publish or store the track info, do so here:
        # e.g., self.publish_person_detection(...)  

    def detect_object(self, image):
        """
        Prepares the image and runs inference on the ONNX model.
        
        Returns:
            (boxes, scores, class_ids)
            - boxes in shape Nx4
            - scores in shape Nx
            - class_ids in shape Nx
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
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self.input_width, self.input_height)).astype(np.float32)
        resized /= 255.0
        return resized.transpose(2, 0, 1)[None]

    def process_output(self, model_output):
        """
        Interprets the raw model output to filter boxes, scores, classes, 
        apply NMS, then keep only 'person' (class_id == 0) for this example.
        """
        preds = np.squeeze(model_output[0]).T  # [num_boxes, 4 + #classes]
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
        boxes, conf_scores, class_ids = boxes[keep_idx], conf_scores[keep_idx], class_ids[keep_idx]

        # Filter only 'person' (COCO class 0). Remove if you want all classes
        # If you'd rather track everything, comment out lines below
        is_person = (class_ids == 0)
        boxes = boxes[is_person]
        conf_scores = conf_scores[is_person]
        class_ids = class_ids[is_person]

        return boxes, conf_scores, class_ids

    def rescale_boxes(self, boxes):
        """
        Convert from model input scale to the original image scale.
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
        """Convert [x_center, y_center, w, h] -> [x1, y1, x2, y2]."""
        x, y, w, h = [boxes[:, i].copy() for i in range(4)]
        boxes[:, 0] = x - w / 2
        boxes[:, 1] = y - h / 2
        boxes[:, 2] = x + w / 2
        boxes[:, 3] = y + h / 2
        return boxes

    def multiclass_nms(self, boxes, scores, class_ids, iou_threshold):
        """Perform NMS per class, gather kept indices."""
        final_keep = []
        for cid in np.unique(class_ids):
            idx = np.where(class_ids == cid)[0]
            keep = self.nms(boxes[idx], scores[idx], iou_threshold)
            final_keep.extend(idx[k] for k in keep)
        return final_keep

    def nms(self, boxes, scores, iou_threshold):
        """Single-class NMS."""
        sorted_idx = np.argsort(scores)[::-1]
        keep = []
        while len(sorted_idx):
            curr = sorted_idx[0]
            keep.append(curr)
            ious = self.compute_iou(boxes[curr], boxes[sorted_idx[1:]])
            sorted_idx = sorted_idx[1:][ious < iou_threshold]
        return keep

    def compute_iou(self, main_box, other_boxes):
        """IoU between one box and an array of boxes."""
        x1 = np.maximum(main_box[0], other_boxes[:, 0])
        y1 = np.maximum(main_box[1], other_boxes[:, 1])
        x2 = np.minimum(main_box[2], other_boxes[:, 2])
        y2 = np.minimum(main_box[3], other_boxes[:, 3])

        inter_w = np.maximum(0, x2 - x1)
        inter_h = np.maximum(0, y2 - y1)
        inter_area = inter_w * inter_h

        box_area = (main_box[2] - main_box[0]) * (main_box[3] - main_box[1])
        other_area = (other_boxes[:, 2] - other_boxes[:, 0]) * (other_boxes[:, 3] - other_boxes[:, 1])

        return inter_area / (box_area + other_area - inter_area + 1e-6)
    
    def draw_tracked_objects(self, frame, tracked_objects):
        """
        Draw bounding boxes for each tracked object. 
        'tracked_objects' is Nx5 = [x1, y1, x2, y2, track_id].
        """
        output_img = frame.copy()
        for i, obj in enumerate(tracked_objects):
            x1, y1, x2, y2, track_id = obj
            # Use a color based on the index or track ID
            color = self.colors[int(track_id) % len(self.colors)]
            p1 = (int(x1), int(y1))
            p2 = (int(x2), int(y2))
            cv2.rectangle(output_img, p1, p2, color, 2)

            label_str = f"ID: {int(track_id)}"
            # You could also display the confidence if desired
            # or a "person" label if you only track persons
            cv2.putText(output_img, label_str, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return output_img

    def spin(self):
        """Main loop for ROS callbacks and display."""
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.latest_frame is not None:
                cv2.imshow("Person Detection YOLOv8", self.latest_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.signal_shutdown("User requested shutdown")

            if self.verbose_mode:
                self.display_depth_image()

            rate.sleep()
        cv2.destroyAllWindows()