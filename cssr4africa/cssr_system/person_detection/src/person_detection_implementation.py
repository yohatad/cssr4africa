import cv2
import rospy
import os
import rospkg
import numpy as np
import onnxruntime
import multiprocessing
import json
import random
from sensor_msgs.msg import Image, CompressedImage
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge, CvBridgeError
from cssr_system.msg import msg_file
from person_detection_tracking import Sort
from geometry_msgs.msg import Point

class PersonDetectionNode:
    def __init__(self):
        self.pub_people = rospy.Publisher("/personDetection/data", msg_file, queue_size=10)
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.use_compressed = rospy.get_param("/personDetection_config/use_compressed", False)
        self.verbose_mode = rospy.get_param("/personDetection_config/verbose_mode", False)
    
    def subscribe_topics(self):
        camera_type = rospy.get_param("/personDetection_config/camera", "realsense")
        
        if camera_type == "realsense":
            self.rgb_topic = self.extract_topics("RealSenseCameraRGB")
            self.depth_topic = self.extract_topics("RealSenseCameraDepth")
        elif camera_type == "pepper":
            self.rgb_topic = self.extract_topics("PepperFrontCamera")
            self.depth_topic = self.extract_topics("PepperDepthCamera")
        elif camera_type == "video":
            self.rgb_topic = self.extract_topics("RealSenseCameraRGB")
            self.depth_topic = self.extract_topics("RealSenseCameraDepth") 
        else:
            rospy.logerr("Invalid camera type specified")
            rospy.signal_shutdown("Invalid camera type")
            return
        
        if not self.rgb_topic or not self.depth_topic:
            rospy.logerr("Camera topic not found.")
            rospy.signal_shutdown("Camera topic not found")
            return

        if self.use_compressed and camera_type == "realsense":
            color_sub = Subscriber(self.rgb_topic + "/compressed", CompressedImage)
            depth_sub = Subscriber(self.depth_topic + "/compressedDepth", CompressedImage)
            rospy.loginfo(f"Subscribed to {self.rgb_topic}/compressed")
            rospy.loginfo(f"Subscribed to {self.depth_topic}/compressedDepth")
        elif self.use_compressed and camera_type == "pepper":
            # There is no compressed topic for Pepper cameras
            rospy.logwarn("Compressed images are not available for Pepper cameras.")
            color_sub = Subscriber(self.rgb_topic, Image)
            depth_sub = Subscriber(self.depth_topic, Image)
            rospy.loginfo(f"Subscribed to {self.rgb_topic}")
            rospy.loginfo(f"Subscribed to {self.depth_topic}")

        elif camera_type == "video":
            color_sub = Subscriber(self.rgb_topic + "/compressed", CompressedImage)
            depth_sub = Subscriber(self.depth_topic + "/compressedDepth", CompressedImage)
            rospy.loginfo(f"Subscribed to {self.rgb_topic}/compressed")
            rospy.loginfo(f"Subscribed to {self.depth_topic}/compressedDepth")

        else:
            color_sub = Subscriber(self.rgb_topic, Image)
            depth_sub = Subscriber(self.depth_topic, Image)
            rospy.loginfo(f"Subscribed to {self.rgb_topic}")
            rospy.loginfo(f"Subscribed to {self.depth_topic}")

        # ApproximateTimeSynchronizer setup
        ats = ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.1
        )
        ats.registerCallback(self.synchronized_callback)

    def synchronized_callback(self, color_data, depth_data):
        try:
            # --- Color Image Processing ---
            if isinstance(color_data, CompressedImage):
                np_arr = np.frombuffer(color_data.data, np.uint8)
                self.color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                self.color_image = self.bridge.imgmsg_to_cv2(color_data, desired_encoding="bgr8")

            # --- Depth Image Processing ---
            if isinstance(depth_data, CompressedImage):
                # Ensure depth_data.format is valid before accessing it
                if hasattr(depth_data, "format") and depth_data.format and "compressedDepth png" in depth_data.format:
                    try:
                        # Handle PNG compression in compressedDepth format
                        depth_header_size = 12
                        depth_img_data = depth_data.data[depth_header_size:]
                        np_arr = np.frombuffer(depth_img_data, np.uint8)
                        depth_img = cv2.imdecode(np_arr, cv2.IMREAD_ANYDEPTH)

                        if depth_img is not None:
                            self.depth_image = depth_img
                        else:
                            rospy.logerr("Failed to decode PNG depth image")
                    except Exception as e:
                        rospy.logerr(f"Depth decoding error: {str(e)}")
                else:
                    # Regular compressed image
                    np_arr = np.frombuffer(depth_data.data, np.uint8)
                    self.depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            else:
                self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding="passthrough")

            if self.color_image is None or self.depth_image is None:
                rospy.logwarn("synchronized_callback: Decoded images are None.")
                return

            # Process synchronized images
            self.process_images()

        except CvBridgeError as e:
            rospy.logerr(f"synchronized_callback CvBridge Error: {str(e)}")
        except Exception as e:
            rospy.logerr(f"synchronized_callback Exception: {str(e)}")

    def check_camera_resolution(self, color_image, depth_image):
        """Check if the color and depth images have the same resolution."""
        if color_image is None or depth_image is None:
            rospy.logwarn("check_camera_resolution: One of the images is None")
            return False
        rgb_h, rgb_w = color_image.shape[:2]
        depth_h, depth_w = depth_image.shape[:2]
        return rgb_h == depth_h and rgb_w == depth_w
        
    @staticmethod
    def read_json_file():
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('cssr_system')
            config_path = os.path.join(package_path, 'person_detection/config', 'person_detection_configuration.json')
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    data = json.load(file)
                    return data
            else:
                rospy.logerr(f"read_json_file: Configuration file not found at {config_path}")
        
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"ROS package 'person_detection' not found: {e}") 

    @staticmethod
    def extract_topics(image_topic):
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('cssr_system')
            config_path = os.path.join(package_path, 'person_detection/data', 'pepper_topics.dat')

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

    def image_callback(self, data):
        """Callback to receive the raw color image."""
        try:
            # Convert the ROS Image message to a NumPy array
            self.color_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.process_images()
        except CvBridgeError as e:
            rospy.logerr("image_callback: CvBridge Error: {}".format(e))
    
    def process_images(self):
        """Process both color and depth images when available."""
        if self.color_image is not None and self.depth_image is not None:
            # Check if resolution matches
            if self.check_camera_resolution(self.color_image, self.depth_image):
                # Process the image with the person detection algorithm
                frame = self.color_image.copy()
                boxes, scores, class_ids = self.detect_object(frame) if hasattr(self, 'detect_object') else ([], [], [])
                
                # If boxes are returned, process them
                if hasattr(self, 'tracker') and len(boxes) > 0:
                    detections = np.hstack([boxes, scores.reshape(-1, 1)])
                    tracked_objects = self.tracker.update(detections)
                    self.latest_frame = self.draw_tracked_objects(frame, tracked_objects)
                    
                    # Prepare and publish tracking data
                    tracking_data = self.prepare_tracking_data(tracked_objects)
                    self.publish_person_detection(tracking_data)
                else:
                    self.latest_frame = frame
            else:
                rospy.logwarn("process_images: Color and depth image resolutions do not match")

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
        """Get the depth value at the centroid of a person."""
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
            return depth_value / 1000.0  # Convert to meters
        else:
            return None

    def get_depth_in_region(self, centroid_x, centroid_y, box_width, box_height, region_scale=0.1):
        """
        Get the depth value within a scaled region around the centroid of a bounding box.

        Args:
            centroid_x (float): The x-coordinate of the centroid.
            centroid_y (float): The y-coordinate of the centroid.
            box_width (float): The width of the bounding box.
            box_height (float): The height of the bounding box.
            region_scale (float): The fraction of the bounding box to consider (default is 0.1).

        Returns:
            float: The average depth value in meters within the scaled region, or None if invalid.
        """
        if self.depth_image is None:
            return None

        # Calculate scaled region dimensions
        region_width = int(box_width * region_scale)
        region_height = int(box_height * region_scale)

        # Calculate the top-left corner of the scaled region
        x_start = int(round(centroid_x - region_width / 2))
        y_start = int(round(centroid_y - region_height / 2))

        # Calculate the bottom-right corner of the scaled region
        x_end = x_start + region_width
        y_end = y_start + region_height

        # Get image dimensions
        image_height, image_width = self.depth_image.shape[:2]

        # Ensure the region is within bounds
        x_start = max(0, x_start)
        y_start = max(0, y_start)
        x_end = min(image_width, x_end)
        y_end = min(image_height, y_end)

        # If the region is invalid (e.g., zero area), return None
        if x_start >= x_end or y_start >= y_end:
            rospy.logwarn(f"Invalid region coordinates ({x_start}, {y_start}, {x_end}, {y_end}).")
            return None

        # Extract the region of interest
        depth_roi = self.depth_image[y_start:y_end, x_start:x_end]

        # Filter out invalid depth values (e.g., zeros or NaNs)
        valid_depth_values = depth_roi[np.isfinite(depth_roi) & (depth_roi > 0)]

        if valid_depth_values.size > 0:
            # Calculate the average depth and convert to meters
            average_depth_in_meters = np.mean(valid_depth_values) / 1000.0
            return average_depth_in_meters
        else:
            return None

    def generate_dark_color(self):
        """Generate a dark color that is visible on a white background."""
        while True:
            color = (random.randint(0, 150), random.randint(0, 150), random.randint(0, 150))  # Dark colors (0-150)
            brightness = (0.299 * color[0] + 0.587 * color[1] + 0.114 * color[2])  # Perceived brightness
            if brightness < 130:  # Ensure the color is dark enough
                return color

    def prepare_tracking_data(self, tracked_objects):
        """Prepare tracking data for publishing."""
        tracking_data = []
        for obj in tracked_objects:
            x1, y1, x2, y2, track_id = obj
            width = x2 - x1
            height = y2 - y1
            centroid_x = (x1 + x2) / 2
            centroid_y = (y1 + y2) / 2
            
            # Get depth at the centroid or in the region
            depth = self.get_depth_in_region(centroid_x, centroid_y, width, height)
            
            point = Point(x=float(centroid_x), y=float(centroid_y), z=float(depth) if depth else 0.0)
            
            tracking_data.append({
                'track_id': str(int(track_id)),
                'centroid': point,
                'width': float(width),
                'height': float(height)
            })
        
        return tracking_data

    def publish_person_detection(self, tracking_data):
        """Publish the detected people to the topic."""
        if not tracking_data:
            # Don't publish empty messages
            return
            
        person_msg = msg_file()
        person_msg.person_label_id = [data['track_id'] for data in tracking_data]
        person_msg.centroids = [data['centroid'] for data in tracking_data]
        person_msg.width = [data['width'] for data in tracking_data]
        person_msg.height = [data['height'] for data in tracking_data]
        
        self.pub_people.publish(person_msg)

class YOLOv8(PersonDetectionNode):
    def __init__(self):
        """
        Initializes the ROS node, loads configuration, and subscribes to necessary topics.
        """
        super().__init__()
        
        self.conf_iou_threshold = rospy.get_param("/personDetection_config/confidence_iou_threshold", 0.5)
        self.sort_max_disap = rospy.get_param("/personDetection_config/sort_max_disappeared", 50)
        self.sort_min_hits = rospy.get_param("/personDetection_config/sort_max_hits", 3)
        self.sort_iou_threshold = rospy.get_param("/personDetection_config/sort_iou_threshold", 0.5)

        # Initialize model
        if not self._init_model():
            rospy.signal_shutdown("Failed to initialize ONNX model")
            return

        # Generate a random color palette for bounding box drawing
        if not hasattr(self, "person_colors"):
            self.person_colors = {}

        # Instantiate SORT tracker
        self.tracker = Sort(max_age=self.sort_max_disap, min_hits=self.sort_min_hits, iou_threshold=self.sort_iou_threshold)

        self.latest_frame = None
        
        # Timer for printing message every 10 seconds
        self.timer = rospy.get_time()
        
        if self.verbose_mode:
            rospy.loginfo("Person Detection YOLOv8 node initialized")
        
        self.subscribe_topics()
        
        # Check camera resolution if both images are available
        if self.depth_image is not None and self.color_image is not None:
            if not self.check_camera_resolution(self.color_image, self.depth_image):
                rospy.logerr("Color camera and depth camera have different resolutions.")
                rospy.signal_shutdown("Resolution mismatch")

    def _init_model(self):
        """Loads the ONNX model and prepares the runtime session."""
        try:
            so = onnxruntime.SessionOptions()
            so.intra_op_num_threads = multiprocessing.cpu_count()
            so.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL

            model_path = rospkg.RosPack().get_path('cssr_system') + '/person_detection/models/yolov8s.onnx'
            self.session = onnxruntime.InferenceSession(model_path, sess_options=so, providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
            )

            active_providers = self.session.get_providers()
            if self.verbose_mode:
                rospy.loginfo(f"Active providers: {active_providers}")
            if "CUDAExecutionProvider" not in active_providers:
                rospy.logwarn("CUDAExecutionProvider is not available. Running on CPU may slow down inference.")
            else:
                if self.verbose_mode:
                    rospy.loginfo("CUDAExecutionProvider is active. Running on GPU for faster inference.")

            input_shape = self.session.get_inputs()[0].shape  # [N, C, H, W]
            self.input_height, self.input_width = input_shape[2], input_shape[3]

            if self.verbose_mode:
                rospy.loginfo(f"ONNX model loaded successfully.")
            
            return True
        except Exception as e:
            rospy.logerr(f"Failed to initialize ONNX model: {e}")
            return False

    def detect_object(self, image):
        """
        Prepares the image and runs inference on the ONNX model.
        
        Returns:
            (boxes, scores, class_ids)
            - boxes in shape Nx4
            - scores in shape Nx1
            - class_ids in shape Nx1
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
        mask = conf_scores > self.conf_iou_threshold
        preds, conf_scores = preds[mask], conf_scores[mask]

        if not len(conf_scores):
            return np.array([]), np.array([]), np.array([])

        class_ids = np.argmax(preds[:, 4:], axis=1)
        boxes = preds[:, :4]
        boxes = self.rescale_boxes(boxes)
        boxes = self.xywh2xyxy(boxes)

        keep_idx = self.multiclass_nms(boxes, conf_scores, class_ids, self.conf_iou_threshold)
        boxes, conf_scores, class_ids = boxes[keep_idx], conf_scores[keep_idx], class_ids[keep_idx]

        # Filter only 'person' (COCO class 0)
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
        for obj in tracked_objects:
            x1, y1, x2, y2, track_id = obj
            track_id = int(track_id)
            
            # Assign a unique color for each track ID
            if track_id not in self.person_colors:
                self.person_colors[track_id] = self.generate_dark_color()
            
            color = self.person_colors[track_id]
            p1 = (int(x1), int(y1))
            p2 = (int(x2), int(y2))
            cv2.rectangle(output_img, p1, p2, color, 2)

            label_str = f"Person: {track_id}"
            cv2.putText(output_img, label_str, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        return output_img

    def spin(self):
        """Main loop for ROS callbacks and display."""
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # Print message every 10 seconds
            if rospy.get_time() - self.timer > 10:
                rospy.loginfo("person_detection: running.")
                self.timer = rospy.get_time()
                
            if self.latest_frame is not None and self.verbose_mode:
                cv2.imshow("Person Detection YOLOv8", self.latest_frame)
                
            if self.verbose_mode:
                self.display_depth_image()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User requested shutdown")

            rate.sleep()
        cv2.destroyAllWindows()