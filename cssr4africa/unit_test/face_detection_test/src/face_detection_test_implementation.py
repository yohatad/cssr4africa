import rospkg
import json
import os
import rospy
import cv2
import time
import threading
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

class FaceDetectionTest:
    def __init__(self):
        """
        Initialize the FaceDetectionTest class.
        Sets up configuration, subscribes to camera topics, and prepares for video/image capture.
        """
        self.rospack = rospkg.RosPack()
        try:
            self.face_detection_test_package_path = self.rospack.get_path('face_detection_test')
            self.face_detection_package_path = self.rospack.get_path('face_detection')
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"ROS package not found: {e}")
            raise RuntimeError(f"Required ROS package not found: {e}")

        # Read the configuration file
        self.config = self.read_json_file()
        if not self.config:
            rospy.logerr("Failed to load configuration. Exiting.")
            raise RuntimeError("Configuration file could not be loaded.")
        
        # Get the algorithm to test from the configuration file
        self.algorithm = self.config.get("algorithm")
        if not self.algorithm:
            rospy.logwarn("No algorithm specified in config, using default")
            self.algorithm = "default"

        # Update the configuration file with the algorithm to test
        self.update_json_file({"algorithm": self.algorithm, "verbose_mode": True})
        rospy.set_param('face_detection_test_config', self.config)

        # Initialize ROS topic subscription and CvBridge
        self.bridge = CvBridge()
        
        # Use locks to protect shared resources
        self.rgb_frames_lock = threading.Lock()
        self.depth_frames_lock = threading.Lock()
        
        self.rgb_frames = []        # For saving RGB video
        self.depth_frames = []      # For saving Depth video
        
        # Configuration parameters with defaults
        self.video_duration = self.config.get("video_duration", 10)  # Default: 10 seconds
        self.image_interval = self.config.get("image_interval", 5)   # Default: 5 seconds
        self.max_frames_buffer = self.config.get("max_frames_buffer", 300)  # Default: ~10s at 30fps
        
        # Timing variables
        self.start_time = None
        self.image_save_time = None
        
        # Video writer objects
        self.rgb_writer = None
        self.depth_writer = None
        
        # Initialize subscribers
        self.camera_param_name = '/faceDetection/camera'
        if not rospy.has_param(self.camera_param_name):
            self.camera_param_name = '/faceDetectionTest/camera'
            if not rospy.has_param(self.camera_param_name):
                rospy.logwarn("Camera parameter not found. Defaulting to 'realsense'")
                rospy.set_param(self.camera_param_name, "realsense")
                
        if rospy.get_param(self.camera_param_name) in ["realsense", "pepper"]:
            self.subscribe_topics()
        else:
            rospy.logerr(f"Unsupported camera type: {rospy.get_param(self.camera_param_name)}")
            raise ValueError(f"Unsupported camera type: {rospy.get_param(self.camera_param_name)}")
            
        rospy.on_shutdown(self.shutdown_hook)

    def extract_topics(self, image_topic):
        """
        Extract topic names from configuration file.
        
        Args:
            image_topic (str): Key to look for in the topics file.
            
        Returns:
            str: The topic name or None if not found.
        """
        try:
            package_path = self.rospack.get_path('face_detection_test')
            config_path = os.path.join(package_path, 'data', 'pepper_topics.dat')

            if not os.path.exists(config_path):
                rospy.logerr(f"extract_topics: Data file not found at {config_path}")
                return None
                
            with open(config_path, 'r') as file:
                for line in file:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    try:
                        key, value = line.split(maxsplit=1)
                        if key.lower() == image_topic.lower():
                            return value
                    except ValueError:
                        rospy.logwarn(f"Invalid line format in topics file: {line}")
                        continue
                        
            rospy.logwarn(f"Topic '{image_topic}' not found in config file")
            return None
            
        except Exception as e:
            rospy.logerr(f"Error in extract_topics: {e}")
            return None

    def read_json_file(self):
        """
        Read the configuration file and return the parsed JSON object.
        
        Returns:
            dict: Configuration dictionary or None if file couldn't be read.
        """
        try:
            config_path = os.path.join(self.face_detection_test_package_path, 'config', 'face_detection_test_configuration.json')
            if not os.path.exists(config_path):
                rospy.logerr(f"read_json_file: Configuration file not found at {config_path}")
                return None
                
            with open(config_path, 'r') as file:
                config = json.load(file)
                return config
                
        except json.JSONDecodeError as e:
            rospy.logerr(f"read_json_file: Error decoding JSON file: {e}")
            return None
        except Exception as e:
            rospy.logerr(f"read_json_file: Unexpected error: {e}")
            return None

    def update_json_file(self, updates):
        """
        Update multiple key-value pairs in the JSON configuration file.

        Args:
            updates (dict): Dictionary of key-value pairs to update.

        Returns:
            bool: True if update was successful, False otherwise.
        """
        try:
            config_path = os.path.join(self.face_detection_package_path, 'config', 'face_detection_configuration.json')

            # Check if file exists
            if not os.path.exists(config_path):
                rospy.logerr(f"update_json_file: Configuration file not found at {config_path}")
                return False

            # Read existing JSON file
            with open(config_path, 'r') as file:
                try:
                    config = json.load(file)
                except json.JSONDecodeError as e:
                    rospy.logerr(f"update_json_file: Error decoding JSON file: {e}")
                    return False

            # Track which keys were updated and which were added
            updated_keys = []
            added_keys = []

            # Update existing keys and add new ones
            for key, value in updates.items():
                if key in config:
                    config[key] = value
                    updated_keys.append(key)
                else:
                    # Log addition of new keys but still add them
                    config[key] = value
                    added_keys.append(key)

            # Write the updated data back to the JSON file
            with open(config_path, 'w') as file:
                json.dump(config, file, indent=4)  # Pretty formatting for readability

            if updated_keys:
                rospy.loginfo(f"update_json_file: Successfully updated keys: {updated_keys}")
            if added_keys:
                rospy.loginfo(f"update_json_file: Added new keys: {added_keys}")
            
            return True

        except Exception as e:
            rospy.logerr(f"update_json_file: Unexpected error: {e}")
            return False

    def subscribe_topics(self):
        """
        Subscribe to RGB and depth camera topics based on the configured camera type.
        Can use either separate callbacks or synchronized callbacks based on configuration.
        """
        camera_type = rospy.get_param(self.camera_param_name)
        
        if camera_type == "realsense":
            self.rgb_topic = self.extract_topics("RealSenseCameraRGB")
            self.depth_topic = self.extract_topics("RealSenseCameraDepth")
        elif camera_type == "pepper":
            self.rgb_topic = self.extract_topics("PepperFrontCamera")
            self.depth_topic = self.extract_topics("PepperDepthCamera")
        else:
            rospy.logerr(f"subscribe_topics: Invalid camera type: {camera_type}")
            return

        if not self.rgb_topic or not self.depth_topic:
            rospy.logerr("subscribe_topics: Camera topic(s) not found.")
            rospy.signal_shutdown("subscribe_topics: Camera topic(s) not found")
            return

        rospy.loginfo(f"Subscribing to RGB topic: {self.rgb_topic}")
        rospy.loginfo(f"Subscribing to depth topic: {self.depth_topic}")
        
        # Check if synchronized callbacks are enabled
        if self.config.get("use_synchronized_callbacks", False):
            self._setup_synchronized_subscribers()
        else:
            self._setup_separate_subscribers()
            
    def _setup_synchronized_subscribers(self):
        """
        Set up synchronized subscribers for RGB and depth images to ensure temporal alignment.
        """
        rgb_sub = Subscriber(self.rgb_topic, Image)
        depth_sub = Subscriber(self.depth_topic, Image)
        
        # Create an approximate time synchronizer
        # queue_size: how many sets of messages to store
        # slop: how close in time the messages need to be (in seconds)
        sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
        sync.registerCallback(self.synchronized_callback)
        
        rospy.loginfo("Set up synchronized RGB and depth image subscribers")
        
    def _setup_separate_subscribers(self):
        """
        Set up separate subscribers for RGB and depth images.
        """
        self.image_sub = rospy.Subscriber(
            self.rgb_topic, 
            Image, 
            self.image_callback,
            queue_size=10
        )
        
        self.depth_sub = rospy.Subscriber(
            self.depth_topic, 
            Image, 
            self.depth_callback,
            queue_size=10
        )
        
        rospy.loginfo("Set up separate RGB and depth image subscribers")

    def synchronized_callback(self, rgb_msg, depth_msg):
        """
        Callback for synchronized RGB and depth images.
        
        Args:
            rgb_msg (sensor_msgs.msg.Image): RGB image message
            depth_msg (sensor_msgs.msg.Image): Depth image message
        """
        try:
            # Convert ROS Image messages to OpenCV format
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            
            # Initialize timing if this is the first frame
            if self.start_time is None:
                self.start_time = time.time()
                self.image_save_time = self.start_time
                
                # Initialize video writers if needed
                if self.config.get("save_video", False):
                    self._initialize_video_writers(cv_rgb.shape, cv_depth.shape)
            
            # Process RGB frame
            self._process_rgb_frame(cv_rgb)
            
            # Process depth frame
            self._process_depth_frame(cv_depth)
            
        except Exception as e:
            rospy.logerr(f"Error in synchronized_callback: {e}")

    def image_callback(self, msg):
        """
        Callback function to process RGB images from the subscribed topic.
        
        Args:
            msg (sensor_msgs.msg.Image): RGB image message
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Start timing when the first frame is received
            if self.start_time is None:
                self.start_time = time.time()
                self.image_save_time = self.start_time
                
                # Initialize video writers if needed
                if self.config.get("save_video", False):
                    height, width = cv_image.shape[:2]
                    self._initialize_rgb_video_writer(width, height)
            
            # Process the RGB frame
            self._process_rgb_frame(cv_image)
            
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def depth_callback(self, msg):
        """
        Callback function to process Depth images from the subscribed topic.
        
        Args:
            msg (sensor_msgs.msg.Image): Depth image message
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # If start_time isn't set yet (no RGB frames received), initialize it
            if self.start_time is None:
                self.start_time = time.time()
                self.image_save_time = self.start_time
                
                # Initialize video writers if needed
                if self.config.get("save_video", False):
                    height, width = cv_depth.shape
                    self._initialize_depth_video_writer(width, height)
            
            # Process the depth frame
            self._process_depth_frame(cv_depth)
            
        except Exception as e:
            rospy.logerr(f"Error in depth_callback: {e}")
            
    def _initialize_video_writers(self, rgb_shape, depth_shape):
        """
        Initialize both RGB and depth video writers.
        
        Args:
            rgb_shape (tuple): Shape of RGB image (height, width, channels)
            depth_shape (tuple): Shape of depth image (height, width)
        """
        rgb_height, rgb_width = rgb_shape[:2]
        depth_height, depth_width = depth_shape
        
        self._initialize_rgb_video_writer(rgb_width, rgb_height)
        self._initialize_depth_video_writer(depth_width, depth_height)
        
    def _initialize_rgb_video_writer(self, width, height):
        """
        Initialize the RGB video writer.
        
        Args:
            width (int): Width of the RGB image
            height (int): Height of the RGB image
        """
        timestamp = int(self.start_time)
        rgb_video_path = os.path.join(
            self.face_detection_test_package_path, 
            'data', 
            f'captured_rgb_video_{timestamp}.mp4'
        )
        
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.rgb_writer = cv2.VideoWriter(
            rgb_video_path, 
            fourcc, 
            30, 
            (width, height), 
            True  # isColor=True for RGB
        )
        
        rospy.loginfo(f"Initialized RGB video writer: {rgb_video_path}")
        
    def _initialize_depth_video_writer(self, width, height):
        """
        Initialize the depth video writer.
        
        Args:
            width (int): Width of the depth image
            height (int): Height of the depth image
        """
        timestamp = int(self.start_time)
        depth_video_path = os.path.join(
            self.face_detection_test_package_path, 
            'data', 
            f'captured_depth_video_{timestamp}.mp4'
        )
        
        if self.config.get("save_depth_as_16bit", False):
            # For 16-bit depth data, use a different approach
            # This is a placeholder - actual implementation depends on specific needs
            self.depth_data_path = os.path.join(
                self.face_detection_test_package_path, 
                'data', 
                f'depth_data_{timestamp}'
            )
            os.makedirs(self.depth_data_path, exist_ok=True)
            rospy.loginfo(f"Will save raw depth frames to: {self.depth_data_path}")
        else:
            # For visualization, we'll convert to 8-bit
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.depth_writer = cv2.VideoWriter(
                depth_video_path, 
                fourcc, 
                30, 
                (width, height), 
                True  # We'll convert depth to BGR for visualization
            )
            
            rospy.loginfo(f"Initialized depth video writer: {depth_video_path}")

    def _process_rgb_frame(self, cv_image):
        """
        Process an RGB frame - save to video and/or as individual image.
        
        Args:
            cv_image (numpy.ndarray): RGB image as a numpy array
        """
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Store frames for video if enabled
        if self.config.get("save_video", False):
            if self.rgb_writer is not None:
                # Write directly to video
                self.rgb_writer.write(cv_image)
            else:
                # Buffer frames if writer not initialized
                with self.rgb_frames_lock:
                    self.rgb_frames.append(cv_image)
                    # Prevent buffer from growing too large
                    if len(self.rgb_frames) > self.max_frames_buffer:
                        self.rgb_frames.pop(0)
            
            # Check if the video duration has been reached
            if elapsed_time >= self.video_duration:
                self._finalize_rgb_video()
        
        # Save individual images at specified intervals
        if (self.config.get("save_image", False) and 
            (current_time - self.image_save_time >= self.image_interval)):
            
            image_path = os.path.join(
                self.face_detection_test_package_path, 
                'data', 
                f'captured_rgb_image_{int(current_time)}.png'
            )
            self.save_image(cv_image, image_path)
            self.image_save_time = current_time
            
    def _process_depth_frame(self, cv_depth):
        """
        Process a depth frame - save to video and/or as individual image.
        
        Args:
            cv_depth (numpy.ndarray): Depth image as a numpy array
        """
        current_time = time.time()
        
        if self.start_time is None:
            # If no RGB frames received yet, we can't calculate elapsed time
            return
            
        elapsed_time = current_time - self.start_time
        
        # Store depth frames for video if enabled
        if self.config.get("save_video", False):
            if self.config.get("save_depth_as_16bit", False):
                # Save raw depth frame as 16-bit PNG
                if hasattr(self, 'depth_data_path'):
                    frame_path = os.path.join(
                        self.depth_data_path, 
                        f'depth_frame_{len(self.depth_frames):06d}.png'
                    )
                    # Use 16-bit PNG for raw depth data
                    cv2.imwrite(frame_path, cv_depth)
                    with self.depth_frames_lock:
                        self.depth_frames.append(frame_path)
            elif self.depth_writer is not None:
                # Normalize and colorize depth for visualization
                depth_colored = self._colorize_depth_for_video(cv_depth)
                self.depth_writer.write(depth_colored)
            else:
                # Buffer frames if writer not initialized
                with self.depth_frames_lock:
                    self.depth_frames.append(cv_depth.copy())
                    # Prevent buffer from growing too large
                    if len(self.depth_frames) > self.max_frames_buffer:
                        self.depth_frames.pop(0)
            
            # Check if the video duration has been reached
            if elapsed_time >= self.video_duration:
                self._finalize_depth_video()
        
        # Save individual images at specified intervals
        if (self.config.get("save_image", False) and 
            (current_time - self.image_save_time >= self.image_interval)):
            
            image_path = os.path.join(
                self.face_detection_test_package_path, 
                'data', 
                f'captured_depth_image_{int(current_time)}.png'
            )
            self.save_image(cv_depth, image_path, is_depth=True)
    
    def _colorize_depth_for_video(self, depth_frame):
        """
        Colorize a depth frame for visualization in video.
        
        Args:
            depth_frame (numpy.ndarray): Raw depth frame
            
        Returns:
            numpy.ndarray: Colorized depth frame suitable for video output
        """
        # Apply colormap for better visualization
        # Normalize to 0-255 range for visualization
        min_val, max_val = self.config.get("depth_min", 0), self.config.get("depth_max", 10000)
        
        # Clip values to specified range
        depth_frame_clipped = np.clip(depth_frame, min_val, max_val)
        
        # Normalize to 0-255 range
        depth_norm = cv2.normalize(
            depth_frame_clipped, 
            None, 
            0, 255, 
            cv2.NORM_MINMAX, 
            cv2.CV_8U
        )
        
        # Apply colormap
        colormap = self.config.get("depth_colormap", cv2.COLORMAP_JET)
        depth_colored = cv2.applyColorMap(depth_norm, colormap)
        
        return depth_colored
            
    def _finalize_rgb_video(self):
        """Finalize RGB video recording and reset for next capture."""
        with self.rgb_frames_lock:
            if self.rgb_writer is None and self.rgb_frames:
                # If we've been buffering frames, write them now
                if not self.rgb_frames:
                    rospy.logwarn("No RGB frames to save.")
                    return
                    
                height, width = self.rgb_frames[0].shape[:2]
                video_path = os.path.join(
                    self.face_detection_test_package_path, 
                    'data', 
                    f'captured_rgb_video_{int(self.start_time)}.mp4'
                )
                
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_writer = cv2.VideoWriter(video_path, fourcc, 30, (width, height), True)
                
                for frame in self.rgb_frames:
                    video_writer.write(frame)
                    
                video_writer.release()
                rospy.loginfo(f"RGB video saved at: {video_path}")
                self.rgb_frames = []
            elif self.rgb_writer is not None:
                # If we were writing directly, just close the writer
                self.rgb_writer.release()
                rospy.loginfo("RGB video recording completed")
                self.rgb_writer = None
                
        # Reset timing for next video
        self.start_time = None
            
    def _finalize_depth_video(self):
        """Finalize depth video recording and reset for next capture."""
        with self.depth_frames_lock:
            if self.config.get("save_depth_as_16bit", False):
                # If we've been saving raw frames, create a metadata file
                if hasattr(self, 'depth_data_path'):
                    metadata = {
                        "num_frames": len(self.depth_frames),
                        "frame_rate": 30,
                        "start_time": self.start_time,
                        "format": "16bit_png"
                    }
                    
                    with open(os.path.join(self.depth_data_path, "metadata.json"), 'w') as f:
                        json.dump(metadata, f, indent=4)
                        
                    rospy.loginfo(f"Raw depth data saved at: {self.depth_data_path}")
                    self.depth_frames = []
                    delattr(self, 'depth_data_path')
            elif self.depth_writer is None and self.depth_frames:
                # If we've been buffering frames, write them now
                if not self.depth_frames:
                    rospy.logwarn("No depth frames to save.")
                    return
                    
                height, width = self.depth_frames[0].shape
                video_path = os.path.join(
                    self.face_detection_test_package_path, 
                    'data', 
                    f'captured_depth_video_{int(self.start_time)}.mp4'
                )
                
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_writer = cv2.VideoWriter(video_path, fourcc, 30, (width, height), True)
                
                for frame in self.depth_frames:
                    colored_frame = self._colorize_depth_for_video(frame)
                    video_writer.write(colored_frame)
                    
                video_writer.release()
                rospy.loginfo(f"Depth video saved at: {video_path}")
                self.depth_frames = []
            elif self.depth_writer is not None:
                # If we were writing directly, just close the writer
                self.depth_writer.release()
                rospy.loginfo("Depth video recording completed")
                self.depth_writer = None
                
        # Note: We don't reset start_time here as it's done in _finalize_rgb_video

    def save_video(self, frames, output_path, is_depth=False):
        """
        Save video frames as an MP4 file.
        
        Args:
            frames (list): List of video frames (images as numpy arrays).
            output_path (str): Path to save the video file.
            is_depth (bool): Flag to indicate if the frames are depth images.
        """
        try:
            if not frames:
                rospy.logwarn("No frames to save.")
                return

            if is_depth and self.config.get("save_depth_as_16bit", False):
                # Create a directory to store individual depth frames
                frames_dir = output_path.replace('.mp4', '_frames')
                os.makedirs(frames_dir, exist_ok=True)
                
                # Save each frame as a 16-bit PNG
                for i, frame in enumerate(frames):
                    frame_path = os.path.join(frames_dir, f"frame_{i:06d}.png")
                    cv2.imwrite(frame_path, frame)
                
                # Save metadata
                metadata = {
                    "num_frames": len(frames),
                    "frame_rate": 30,
                    "format": "16bit_png"
                }
                
                with open(os.path.join(frames_dir, "metadata.json"), 'w') as f:
                    json.dump(metadata, f, indent=4)
                    
                rospy.loginfo(f"Raw depth frames saved at: {frames_dir}")
                return

            height, width = frames[0].shape if is_depth else frames[0].shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(output_path, fourcc, 30, (width, height), not is_depth)

            for frame in frames:
                if is_depth:
                    # Create a more visually informative depth visualization
                    frame = self._colorize_depth_for_video(frame)
                video_writer.write(frame)

            video_writer.release()
            rospy.loginfo(f"Video saved successfully at: {output_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save video: {e}")

    def save_image(self, image, output_path, is_depth=False):
        """
        Save an image as a PNG file.
        
        Args:
            image (numpy array): The image to save.
            output_path (str): Path to save the image file.
            is_depth (bool): Flag to indicate if the image is a depth image.
        """
        try:
            if image is None:
                rospy.logwarn("No image to save.")
                return

            if is_depth:
                if self.config.get("save_depth_as_16bit", False):
                    # Save as 16-bit image for depth
                    cv2.imwrite(output_path, image)
                else:
                    # Create a normalized 8-bit visualization for viewing
                    vis_path = output_path.replace('.png', '_vis.png')
                    depth_vis = self._colorize_depth_for_video(image)
                    
                    # Save both raw depth and visualization
                    cv2.imwrite(output_path, image)
                    cv2.imwrite(vis_path, depth_vis)
                    rospy.loginfo(f"Depth image saved at: {output_path} (raw) and {vis_path} (visualization)")
                    return
            else:
                cv2.imwrite(output_path, image)
                
            rospy.loginfo(f"Image saved successfully at: {output_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save image: {e}")
            
    def shutdown_hook(self):
        """
        Cleanup resources when ROS is shutting down.
        """
        # Close video writers if they're open
        if hasattr(self, 'rgb_writer') and self.rgb_writer is not None:
            self.rgb_writer.release()
            rospy.loginfo("RGB video writer closed on shutdown")
            
        if hasattr(self, 'depth_writer') and self.depth_writer is not None:
            self.depth_writer.release()
            rospy.loginfo("Depth video writer closed on shutdown")
            
        # Unsubscribe from topics
        if hasattr(self, 'image_sub'):
            self.image_sub.unregister()
            
        if hasattr(self, 'depth_sub'):
            self.depth_sub.unregister()
            
        rospy.loginfo("FaceDetectionTest shutdown complete")