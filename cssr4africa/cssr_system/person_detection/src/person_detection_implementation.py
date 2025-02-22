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
    def __init__(self, config):
        if config is not None:
            self.config = config
        else:
            self.config = self.read_json_file()

        self.pub_people = rospy.Publisher("/personDetection/data", person_detection, queue_size=10)
        self.bridge = CvBridge()
        self.depth_image = None
    
    def subscribe_topics(self):
        camera_type = self.config.get("camera", "realsense")

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
    def parse_config():
        config = {}
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('person_detection')
            config_path = os.path.join(package_path, 'config', 'person_detection_configuration.ini')
            
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    for line in file:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            continue
                        
                        parts = line.split(maxsplit=1)
                        if len(parts) != 2:
                            print(f"Invalid configuration line: '{line}'")
                            continue
                        key, value = parts
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
            print(f"\033[91mROS package 'person_detection' not found: {e}\033[0m")
        
        return config
    
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
                print(f"\033[91mData file not found at {config_path}\033[0m")
        except rospkg.ResourceNotFound as e:
            print(f"\033[91mROS package 'person_detection' not found: {e}\033[0m")

    def depth_callback(self, data):
        """Callback to receive the depth image."""
        try:
            # Convert the ROS Image message to a NumPy array
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))

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

class YOLOv8ROS(PersonDetectionNode):
    def __init__(self, config):
        super(YOLOv8ROS, self).__init__(config)

        model_path = 'package://person_detection/models/yolov8s.onnx'
        yolov8_ros = self.resolve_model_path(model_path)

        # Initialize SixDrepNet ONNX session
        try:
            session_option = onnxruntime.SessionOptions()
            session_option.log_severity_level = 3
            session_option.intra_op_num_threads = multiprocessing.cpu_count()
            session_option.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.session = onnxruntime.InferenceSession(
                yolov8_ros,
                sess_options=session_option,
                providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
            )

            active_providers = self.session.get_providers()
            rospy.loginfo(f"Active providers: {active_providers}")
            if "CUDAExecutionProvider" not in active_providers:
                rospy.logwarn("CUDAExecutionProvider is not available. Running on CPU may slow down inference.")
            else:
                rospy.loginfo("CUDAExecutionProvider is active. Running on GPU for faster inference.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize SixDrepNet ONNX session: {e}")
            return  # Exit early if initialization fails
        
        self.get_input_details()
        self.get_output_details()

        self.frame_counter = 0
        self.tracks = []

        self.verbose_mode = self.config.get("verbosemode", False)
        self.latest_frame = None
        
        rospy.loginfo("Person detection node initialized")

        self.subscribe_topics()

    def get_input_details(self):
        model_inputs = self.session.get_inputs()
        self.input_names = [model_inputs[i].name for i in range(len(model_inputs))]

        self.input_shape = model_inputs[0].shape
        self.input_height = self.input_shape[2]
        self.input_width = self.input_shape[3]

    def get_output_details(self):
        model_outputs = self.session.get_outputs()
        self.output_names = [model_outputs[i].name for i in range(len(model_outputs))]

    def detect_object(self, image):
        # Preprocess the input image
        processed_frame = self.prepare_input(image)

        # Run inference on the preprocessed frame
        outputs = self.session.run(self.output_names, {self.input_names[0]: processed_frame})

        # Postprocess the results
        self.boxes, self.scores, self.class_ids = self.process_output(outputs)

        return self.boxes, self.scores, self.class_ids
    
    def prepare_input(self, image):
        """
        Preprocess the input image for inference.
        Args:
            image: Input image in OpenCV format.
        """
        
        self.img_height, self.img_width = image.shape[:2]
        input_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Resize input image
        input_img = cv2.resize(input_img, (self.input_width, self.input_height))

        # Normalize the input image
        input_img = input_img.astype(np.float32) / 255.0
        input_img = input_img.transpose(2,0,1)
        input_tensor = input_img[np.newaxis, ...].astype(np.float32)

        return input_tensor
    
    def process_output(self, output):
        """
        Process the output of the model.
        Args:
            output: Output tensor from the model.
        """
        predictions = np.squeeze(output[0]).T

        # Extract the bounding box coordinates
        scores = np.max(predictions[:, 4:], axis=1)
        predictions = predictions[scores > self.config.get("confidence_threshold", 0.5), :]
        scores = scores[scores > self.config.get("confidence_threshold", 0.5)]

        if len(scores) == 0:
            return [], [], []
    
        # Get the class with the highest confidence
        class_ids = np.argmax(predictions[:, 4:], axis=1)

        # Get bounding boxes for each object
        boxes = self.extract_boxes(predictions)

        indices = self.multiclass_nms(boxes, scores, class_ids, self.config.get("iou_threshold", 0.5))

        return boxes[indices], scores[indices], class_ids[indices]
    
    def extract_boxes(self, predictions):
        # Extract boxes from predictions
        boxes = predictions[:, :4]

        # Scale boxes to original image dimensions
        boxes = self.rescale_boxes(boxes)

        # Convert boxes to xyxy format
        boxes = self.xywh2xyxy(boxes)

        return boxes
    
    def rescale_boxes(self, boxes):
        # Rescale boxes to original image dimensions
        input_shape = np.array([self.input_width, self.input_height, self.input_width, self.input_height])
        boxes = np.divide(boxes, input_shape, dtype=np.float32)
        boxes *= np.array([self.img_width, self.img_height, self.img_width, self.img_height])
        return boxes
            
    def nms(self, boxes, scores, iou_threshold):
        # Sort by score
        sorted_indices = np.argsort(scores)[::-1]

        keep_boxes = []
        while sorted_indices.size > 0:
            # Pick the last box
            box_id = sorted_indices[0]
            keep_boxes.append(box_id)

            # Compute IoU of the picked box with the rest
            ious = self.compute_iou(boxes[box_id, :], boxes[sorted_indices[1:], :])

            # Remove boxes with IoU over the threshold
            keep_indices = np.where(ious < iou_threshold)[0]

            # print(keep_indices.shape, sorted_indices.shape)
            sorted_indices = sorted_indices[keep_indices + 1]

        return keep_boxes

    def multiclass_nms(self, boxes, scores, class_ids, iou_threshold):

        unique_class_ids = np.unique(class_ids)

        keep_boxes = []
        for class_id in unique_class_ids:
            class_indices = np.where(class_ids == class_id)[0]
            class_boxes = boxes[class_indices,:]
            class_scores = scores[class_indices]

            class_keep_boxes = self.nms(class_boxes, class_scores, iou_threshold)
            keep_boxes.extend(class_indices[class_keep_boxes])

        return keep_boxes

    def compute_iou(self, box, boxes):
        # Compute xmin, ymin, xmax, ymax for both boxes
        xmin = np.maximum(box[0], boxes[:, 0])
        ymin = np.maximum(box[1], boxes[:, 1])
        xmax = np.minimum(box[2], boxes[:, 2])
        ymax = np.minimum(box[3], boxes[:, 3])

        # Compute intersection area
        intersection_area = np.maximum(0, xmax - xmin) * np.maximum(0, ymax - ymin)

        # Compute union area
        box_area = (box[2] - box[0]) * (box[3] - box[1])
        boxes_area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        union_area = box_area + boxes_area - intersection_area

        # Compute IoU
        iou = intersection_area / union_area

        return iou

    def xywh2xyxy(self, x):
        # Convert bounding box (x, y, w, h) to bounding box (x1, y1, x2, y2)
        y = np.copy(x)
        y[..., 0] = x[..., 0] - x[..., 2] / 2
        y[..., 1] = x[..., 1] - x[..., 3] / 2
        y[..., 2] = x[..., 0] + x[..., 2] / 2
        y[..., 3] = x[..., 1] + x[..., 3] / 2
        return y

    def draw_detections(self, image, boxes, scores, class_ids, mask_alpha=0.3):
        det_img = image.copy()

        img_height, img_width = image.shape[:2]
        font_size = min([img_height, img_width]) * 0.0006
        text_thickness = int(min([img_height, img_width]) * 0.001)

        det_img = self.draw_masks(det_img, boxes, class_ids, mask_alpha)

        # Draw bounding boxes and labels of detections
        for class_id, box, score in zip(class_ids, boxes, scores):
            color = colors[class_id]

            self.draw_box(det_img, box, color)

            label = class_names[class_id]
            caption = f'{label} {int(score * 100)}%'
            self.draw_text(det_img, caption, box, color, font_size, text_thickness)

        return det_img
    
    def draw_box(self, image: np.ndarray, box: np.ndarray, color: tuple[int, int, int] = (0, 0, 255),
                thickness: int = 2) -> np.ndarray:
        x1, y1, x2, y2 = box.astype(int)
        return cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)


    def draw_text(image: np.ndarray, text: str, box: np.ndarray, color: tuple[int, int, int] = (0, 0, 255),
                font_size: float = 0.001, text_thickness: int = 2) -> np.ndarray:
        x1, y1, x2, y2 = box.astype(int)
        (tw, th), _ = cv2.getTextSize(text=text, fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                    fontScale=font_size, thickness=text_thickness)
        th = int(th * 1.2)

        cv2.rectangle(image, (x1, y1),
                    (x1 + tw, y1 - th), color, -1)

        return cv2.putText(image, text, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, font_size, (255, 255, 255), text_thickness, cv2.LINE_AA)

    def draw_masks(self, image: np.ndarray, boxes: np.ndarray, classes: np.ndarray, mask_alpha: float = 0.3) -> np.ndarray:
        mask_img = image.copy()

        # Draw bounding boxes and labels of detections
        for box, class_id in zip(boxes, classes):
            color = colors[class_id]

            x1, y1, x2, y2 = box.astype(int)

            # Draw fill rectangle in mask image
            cv2.rectangle(mask_img, (x1, y1), (x2, y2), color, -1)

        return cv2.addWeighted(mask_img, mask_alpha, image, 1 - mask_alpha, 0)
    
    def draw_detections(self, image, boxes, scores, class_ids, mask_alpha=0.3):
        det_img = image.copy()

        img_height, img_width = image.shape[:2]
        font_size = min([img_height, img_width]) * 0.0006
        text_thickness = int(min([img_height, img_width]) * 0.001)

        det_img = self.draw_masks(det_img, boxes, class_ids, mask_alpha)

        # Draw bounding boxes and labels of detections
        for class_id, box, score in zip(class_ids, boxes, scores):
            color = colors[class_id]

            self.draw_box(det_img, box, color)

            label = class_names[class_id]
            caption = f'{label} {int(score * 100)}%'
            self.draw_text(det_img, caption, box, color, font_size, text_thickness)

        return det_img
    
    def draw_text(self, image: np.ndarray, text: str, box: np.ndarray, color: tuple[int, int, int] = (0, 0, 255),
              font_size: float = 0.001, text_thickness: int = 2) -> np.ndarray:
        x1, y1, x2, y2 = box.astype(int)
        (tw, th), _ = cv2.getTextSize(text=text, fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                    fontScale=font_size, thickness=text_thickness)
        th = int(th * 1.2)

        cv2.rectangle(image, (x1, y1),
                    (x1 + tw, y1 - th), color, -1)

        return cv2.putText(image, text, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, font_size, (255, 255, 255), text_thickness, cv2.LINE_AA)
        
    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run detection on the frame
        boxes, scores, class_ids = self.detect_object(frame)

        # Draw detections on the frame
        if len(boxes) > 0:
            frame_with_detections = self.draw_detections(frame, boxes, scores, class_ids)
        else:
            frame_with_detections = frame

        # Update the latest frame for display in spin()
        self.latest_frame = frame_with_detections

    def spin(self):
        """Main loop to display the processed frames and depth images."""
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.latest_frame is not None:
                # Display the latest processed frame with detections
                cv2.imshow("YOLOv8 ROS", self.latest_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.signal_shutdown("User requested shutdown")

            # Display the depth image if verbose mode is on
            if self.verbose_mode:
                self.display_depth_image()

            rate.sleep()
        cv2.destroyAllWindows()
