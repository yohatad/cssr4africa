""""
face_detection_test_implementation.py Implementation code for running the Face and Mutual Gaze Detection and Localization unit test.

Author: Yohannes Tadesse Haile
Date: March 21, 2025
Version: v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import rospkg
import rospy
import os
import json
import numpy as np
import cv2
import time
import threading
import colorsys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from face_detection_test.msg import msg_file

class FaceDetectionTest:
    def __init__(self):
        """
        Initialize the FaceDetectionTest class.
        Sets up configuration, subscribes to camera topics, and prepares for video/image capture.
        Also configures the face_detection node and subscribes to its outputs.
        """
        self.rospack = rospkg.RosPack()
        try:
            self.face_detection_test_package_path = self.rospack.get_path('unit_test')
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"ROS package not found: {e}")
            raise RuntimeError(f"Required ROS package not found: {e}")

        # Read the configuration file
        self.config = self.read_json_file()
        if not self.config:
            rospy.logerr("Failed to load configuration. Exiting.")
            raise RuntimeError("Configuration file could not be loaded.")
        
        self.camera = rospy.get_param('faceDetection/camera', default='video')

        # Set the algorithm to test based on the configuration
        self.algorithm = self.config.get("algorithm", "sixdrep")
        rospy.set_param('/faceDetection_config/algorithm', self.algorithm)
        
        # Set the verbose mode based on the configuration
        self.verbose_mode = self.config.get("verbose_mode", False)
        rospy.set_param('/faceDetection_config/verbose_mode', self.verbose_mode)

        # Initialize ROS topic subscription and CvBridge
        self.bridge = CvBridge()
        
        # Use locks to protect shared resources
        self.rgb_frames_lock = threading.Lock()
        self.depth_frames_lock = threading.Lock()
        
        self.rgb_frames = []        # For saving RGB video
        self.depth_frames = []      # For saving Depth video
        
        # For storing multiple faces and their data
        self.face_labels = []
        self.face_centroids = []
        self.face_widths = []
        self.face_heights = []
        self.face_mutual_gazes = []
        
        # Generate distinct colors for different faces
        self.face_colors = {}
        
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
                        
        # Only subscribe to camera if recording or visualization is needed
        if (self.config.get("save_video", False) or 
            self.config.get("save_image", False)):
            if self.camera in ["realsense", "pepper"]:
                self.subscribe_camera_topics()
            else:
                rospy.logerr(f"Unsupported camera type: {self.camera}")
                raise ValueError(f"Unsupported camera type: {self.camera}")
        else:
            # Only subscribe to face detection data, not camera feeds
            rospy.loginfo("Camera subscription disabled - not recording or visualizing")
                
        # Subscribe to face detection data
        self.face_data_sub = rospy.Subscriber(
            '/faceDetection/data',
            msg_file,
            self.face_data_callback,
            queue_size=10
        )
        
        # Set a delay for recording if configured
        if self.config.get("recording_delay", 0) > 0:
            delay = self.config.get("recording_delay", 5)  # Default 5 second delay
            if self.verbose_mode:
                rospy.loginfo(f"Will start recording after {delay} seconds delay")
            rospy.Timer(rospy.Duration(delay), self.start_recording_callback, oneshot=True)
            self.recording_enabled = False
        else:
            self.recording_enabled = True
        
        rospy.on_shutdown(self.shutdown_hook)
        
    
    def start_recording_callback(self, event):
        """
        Callback to start recording after the initial delay.
        """
        self.recording_enabled = True
        if self.verbose_mode:
            rospy.loginfo("Recording delay complete - starting to record")

    def face_data_callback(self, msg):
        """
        Process incoming face detection data containing multiple faces.
        
        Args:
            msg: Custom message with face_label_id, centroids, width, height, and mutualGaze arrays
        """
        # Store the face detection data
        self.face_labels = msg.face_label_id
        self.face_centroids = msg.centroids
        self.face_widths = msg.width  # Store the width values
        self.face_heights = msg.height  # Store the height values
        self.face_mutual_gazes = msg.mutualGaze
        
        # Generate colors for new faces
        for face_id in self.face_labels:
            if face_id not in self.face_colors:
                # Generate a visually distinct color for this face
                hue = hash(face_id) % 100 / 100.0  # Use hash to get consistent color for same ID
                rgb = colorsys.hsv_to_rgb(hue, 0.8, 1.0)
                bgr = (int(rgb[2] * 255), int(rgb[1] * 255), int(rgb[0] * 255))  # Convert to BGR
                self.face_colors[face_id] = bgr

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
            config_path = os.path.join(self.face_detection_test_package_path, 'face_detection_test/config', 'face_detection_test_configuration.json')
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

    def subscribe_camera_topics(self):
        """
        Subscribe to RGB and depth camera topics based on the configured camera type.
        Can use either separate callbacks or synchronized callbacks based on configuration.
        """        
        if self.camera == "realsense":
            self.rgb_topic = self.extract_topics("RealSenseCameraRGB")
            self.depth_topic = self.extract_topics("RealSenseCameraDepth")
        elif self.camera == "pepper":
            self.rgb_topic = self.extract_topics("PepperFrontCamera")
            self.depth_topic = self.extract_topics("PepperDepthCamera")
        else:
            rospy.logerr(f"subscribe_camera_topics: Invalid camera type: {self.camera}")
            return

        if not self.rgb_topic or not self.depth_topic:
            rospy.logerr("subscribe_camera_topics: Camera topic(s) not found.")
            rospy.signal_shutdown("subscribe_camera_topics: Camera topic(s) not found")
            return

        rospy.loginfo(f"Subscribed to {self.rgb_topic}")
        rospy.loginfo(f"Subscribed to {self.depth_topic}")
        
        self.setup_synchronized_subscribers()
            
    def setup_synchronized_subscribers(self):
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

        # Print message every 5 seconds
        if rospy.get_time() - self.timer > 10:
            rospy.loginfo("face_detection_test: running.")
            self.timer = rospy.get_time()
        
        rospy.loginfo("Set up synchronized RGB and depth image subscribers")
    
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
                if self.config.get("save_video", False) and self.recording_enabled:
                    self.initialize_video_writers(cv_rgb.shape, cv_depth.shape)
            
            # Process RGB frame with face detection overlay
            self.process_rgb_frame(cv_rgb)
            
            # Process depth frame
            self.process_depth_frame(cv_depth)
            
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
                if self.config.get("save_video", False) and self.recording_enabled:
                    height, width = cv_image.shape[:2]
                    self.initialize_rgb_video_writer(width, height)
            
            # Process the RGB frame with face detection overlay
            self.process_rgb_frame(cv_image)
            
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
                if self.config.get("save_video", False) and self.recording_enabled:
                    height, width = cv_depth.shape
                    self.initialize_depth_video_writer(width, height)
            
            # Process the depth frame
            self.process_depth_frame(cv_depth)
            
        except Exception as e:
            rospy.logerr(f"Error in depth_callback: {e}")
            
    def initialize_video_writers(self, rgb_shape, depth_shape):
        """
        Initialize both RGB and depth video writers.
        
        Args:
            rgb_shape (tuple): Shape of RGB image (height, width, channels)
            depth_shape (tuple): Shape of depth image (height, width)
        """
        rgb_height, rgb_width = rgb_shape[:2]
        depth_height, depth_width = depth_shape
        
        self.initialize_rgb_video_writer(rgb_width, rgb_height)
        self.initialize_depth_video_writer(depth_width, depth_height)
        
    def initialize_rgb_video_writer(self, width, height):
        """
        Initialize the RGB video writer.
        
        Args:
            width (int): Width of the RGB image
            height (int): Height of the RGB image
        """
        timestamp = int(self.start_time)
        rgb_video_path = os.path.join(
            self.face_detection_test_package_path, 'face_detection_test/data', f'face_detection_rgb_video_{timestamp}.mp4')
        
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.rgb_writer = cv2.VideoWriter(
            rgb_video_path, 
            fourcc, 
            30, 
            (width, height), 
            True  # isColor=True for RGB
        )
                
    def initialize_depth_video_writer(self, width, height):
        """
        Initialize the depth video writer.
        
        Args:
            width (int): Width of the depth image
            height (int): Height of the depth image
        """
        timestamp = int(self.start_time)
        depth_video_path = os.path.join(self.face_detection_test_package_path, 'face_detection_test/data', f'face_detection_depth_video_{timestamp}.mp4')
        
        # For visualization, convert to colorized 8-bit
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.depth_writer = cv2.VideoWriter(depth_video_path, fourcc, 30, (width, height), True) # isColor=True for RGB output
        
    def draw_face_detection_overlay(self, image):
        """
        Draw face detection overlay on the image for multiple faces.
        
        Args:
            image (numpy.ndarray): RGB image to draw on
        """
        # Check if we have face data
        if not self.face_labels or not self.face_centroids:
            return
        
        for i, (face_id, centroid, width, height, mutual_gaze) in enumerate(zip(
            self.face_labels, self.face_centroids, self.face_widths, self.face_heights, self.face_mutual_gazes)):
            
            # Get face coordinates
            centroid_x, centroid_y = int(centroid.x), int(centroid.y)
            
            # Use width and height from the message
            face_width = int(width)
            face_height = int(height)
            
            # Calculate bounding box coordinates
            x1 = max(0, centroid_x - face_width // 2)
            y1 = max(0, centroid_y - face_height // 2)
            x2 = min(image.shape[1], centroid_x + face_width // 2)
            y2 = min(image.shape[0], centroid_y + face_height // 2)
            
            # Get color for this face
            face_color = self.face_colors.get(face_id, (0, 255, 0))
            
            # Draw bounding box with assigned color
            cv2.rectangle(image, (x1, y1), (x2, y2), face_color, 2)
            
            # Add labels above bounding box
            label = "Engaged" if mutual_gaze else "Not Engaged"
            cv2.putText(image, label, (x1 + 10, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, face_color, 2)
            
            cv2.putText(image, f"Face: {face_id}", (x1 + 10, y1 - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, face_color, 2)
            
            # Draw depth information below the box
            cv2.putText(
                image,
                f"Depth: {centroid.z:.2f}m",
                (x1 + 10, y2 + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                face_color,
                2
            )
            
            # Draw centroid point
            cv2.circle(image, (centroid_x, centroid_y), 4, (255, 0, 0), -1)

    def process_rgb_frame(self, cv_image):
        """
        Process an RGB frame - draw face detection overlay and save to video/image.
        
        Args:
            cv_image (numpy.ndarray): RGB image as a numpy array
        """
        current_time = time.time()
        elapsed_time = current_time - self.start_time if self.start_time else 0
        
        # Create a copy of the frame for drawing overlays
        display_image = cv_image.copy()
        
        # Draw face detection overlay
        self.draw_face_detection_overlay(display_image)
        
        # Store frames for video if enabled
        if self.config.get("save_video", False) and self.recording_enabled:
            if self.rgb_writer is not None:
                # Write directly to video
                self.rgb_writer.write(display_image)
            else:
                # Buffer frames if writer not initialized
                with self.rgb_frames_lock:
                    self.rgb_frames.append(display_image)
                    # Prevent buffer from growing too large
                    if len(self.rgb_frames) > self.max_frames_buffer:
                        self.rgb_frames.pop(0)
            
            # Check if the video duration has been reached
            if elapsed_time >= self.video_duration:
                self.finalize_rgb_video()
        
        # Save individual images at specified intervals
        if (self.config.get("save_image", False) and 
            self.recording_enabled and
            (current_time - self.image_save_time >= self.image_interval)):
            
            image_path = os.path.join(
                self.face_detection_test_package_path, 
                'face_detection_test/data', 
                f'face_detection_rgb_image_{int(current_time)}.png'
            )
            self.save_image(display_image, image_path)
            self.image_save_time = current_time
            
    def process_depth_frame(self, cv_depth):
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
        if self.config.get("save_video", False) and self.recording_enabled:
            if self.depth_writer is not None:
                # Normalize and colorize depth for visualization
                depth_colored = self.colorize_depth_for_video(cv_depth)
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
                self.finalize_depth_video()
        
        # Save individual images at specified intervals
        if (self.config.get("save_image", False) and 
            self.recording_enabled and
            (current_time - self.image_save_time >= self.image_interval)):
            
            image_path = os.path.join(
                self.face_detection_test_package_path, 
                'face_detection_test/data', 
                f'face_detection_depth_image_{int(current_time)}.png'
            )
            self.save_image(cv_depth, image_path, is_depth=True)
        
    def colorize_depth_for_video(self, depth_frame):
        """
        Colorize a depth frame for visualization in video.
        
        Args:
            depth_frame (numpy.ndarray): Raw depth frame
            
        Returns:
            numpy.ndarray: Colorized depth frame suitable for video output
        """
        # Apply colormap for better visualization
        # Normalize to 0-255 range for visualization
        min_val, max_val = 0, 10000
        
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
        depth_colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        
        return depth_colored
            
    def finalize_rgb_video(self):
        """Finalize RGB video recording and reset for next capture."""
        with self.rgb_frames_lock:
            if self.rgb_writer is None and self.rgb_frames:                    
                height, width = self.rgb_frames[0].shape[:2]
                video_path = os.path.join(
                    self.face_detection_test_package_path, 
                    'face_detection_test/data', 
                    f'face_detection_rgb_video_{int(self.start_time)}.mp4'
                )
                
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_writer = cv2.VideoWriter(video_path, fourcc, 30, (width, height), True)
                
                for frame in self.rgb_frames:
                    video_writer.write(frame)
                    
                video_writer.release()
                if self.verbose_mode:
                    rospy.loginfo(f"RGB video saved at: {video_path}")
                self.rgb_frames = []
            elif self.rgb_writer is not None:
                # If we were writing directly, just close the writer
                self.rgb_writer.release()
                if self.verbose_mode:
                    rospy.loginfo("RGB video recording completed")
                self.rgb_writer = None
                
        # Reset timing for next video
        self.start_time = None
            
    def finalize_depth_video(self):
        """Finalize depth video recording and reset for next capture."""
        with self.depth_frames_lock:
            if self.depth_writer is None and self.depth_frames:                    
                height, width = self.depth_frames[0].shape
                video_path = os.path.join(
                    self.face_detection_test_package_path, 
                    'face_detection_test/data', 
                    f'face_detection_depth_video_{int(self.start_time)}.mp4'
                )
                
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_writer = cv2.VideoWriter(video_path, fourcc, 30, (width, height), True)
                
                for frame in self.depth_frames:
                    colored_frame = self.colorize_depth_for_video(frame)
                    video_writer.write(colored_frame)
                    
                video_writer.release()
                if self.verbose_mode:
                    rospy.loginfo(f"Depth video saved at: {video_path}")
                self.depth_frames = []
            elif self.depth_writer is not None:
                # If we were writing directly, just close the writer
                self.depth_writer.release()
                if self.verbose_mode:
                    rospy.loginfo("Depth video recording completed")
                self.depth_writer = None

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

            height, width = frames[0].shape if is_depth else frames[0].shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(output_path, fourcc, 30, (width, height), not is_depth)

            for frame in frames:
                if is_depth:
                    # Create a more visually informative depth visualization
                    frame = self.colorize_depth_for_video(frame)
                video_writer.write(frame)

            video_writer.release()
            if self.verbose_mode:
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
                # Create a normalized 8-bit visualization for viewing
                vis_path = output_path.replace('.png', '_vis.png')
                depth_vis = self.colorize_depth_for_video(image)
                
                # Save both raw depth and visualization
                cv2.imwrite(output_path, image)
                cv2.imwrite(vis_path, depth_vis)
                if self.verbose_mode:
                    rospy.loginfo(f"Depth image saved at: {output_path} (raw) and {vis_path} (visualization)")
                return
            else:
                cv2.imwrite(output_path, image)

            if self.verbose_mode:    
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
            if self.verbose_mode:
                rospy.loginfo("RGB video writer closed on shutdown")
            
        if hasattr(self, 'depth_writer') and self.depth_writer is not None:
            self.depth_writer.release()
            if self.verbose_mode:
                rospy.loginfo("Depth video writer closed on shutdown")
            
        # Unsubscribe from topics
        if hasattr(self, 'image_sub'):
            self.image_sub.unregister()
            
        if hasattr(self, 'depth_sub'):
            self.depth_sub.unregister()
            
        if hasattr(self, 'face_data_sub'):
            self.face_data_sub.unregister()
            
        if self.verbose_mode:
            rospy.loginfo("FaceDetectionTest shutdown complete")