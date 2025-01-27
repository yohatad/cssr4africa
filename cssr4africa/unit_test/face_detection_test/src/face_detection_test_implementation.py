import rospkg
import json
import os
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
class FaceDetectionTest:
    def __init__(self):
        """
        Initialize the FaceDetectionTest class.
        """
        self._rospack = rospkg.RosPack()
        self._face_detection_test_package_path = self._rospack.get_path('face_detection_test')
        self._face_detection_package_path = self._rospack.get_path('face_detection')

        # Read the configuration file
        self.config = self.read_json_file()
        if not self.config:
            rospy.logerr("Failed to load configuration. Exiting.")
            raise RuntimeError("Configuration file could not be loaded.")

        # Initialize ROS topic subscription and CvBridge
        self.bridge = CvBridge()
        self.rgb_frames = []  # For saving RGB video
        self.depth_frames = []  # For saving Depth video
        self.image_topic = None
        self.start_time = None
        self.image_save_time = None
        self.video_duration = self.config.get("video_duration", 10)  # Default video duration: 10 seconds
        self.image_interval = self.config.get("image_interval", 5)  # Default image interval: 5 seconds


        self.subscribe_topics()

    @staticmethod
    def extract_topics(image_topic):
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('face_detection')
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
            rospy.logerr(f"ROS package 'face_detection' not found: {e}")

    def read_json_file(self):
        """Read the configuration file and return the parsed JSON object."""
        try:
            config_path = os.path.join(self._face_detection_test_package_path, 'config', 'face_detection_test_configuration.json')
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    config = json.load(file)
                    return config
            else:
                rospy.logerr(f"read_json_file: Configuration file not found at {config_path}")
                return None
        except json.JSONDecodeError as e:
            rospy.logerr(f"read_json_file: Error decoding JSON file: {e}")
            return None

    def subscribe_topics(self):
        camera_type = self.config.get("camera")
        if camera_type == "realsense":
            self.rgb_topic = self.extract_topics("RealSenseCameraRGB")
            self.depth_topic = self.extract_topics("RealSenseCameraDepth")
            rospy.loginfo(f"Subscribed to {self.rgb_topic}")
            rospy.loginfo(f"Subscribed to {self.depth_topic}")
        elif camera_type == "pepper":
            self.rgb_topic = self.extract_topics("PepperFrontCamera")
            self.depth_topic = self.extract_topics("PepperDepthCamera")
            rospy.loginfo(f"Subscribed to {self.rgb_topic}")
            rospy.loginfo(f"Subscribed to {self.depth_topic}")
        else:
            rospy.logerr("subscribe_topics: Invalid camera type specified")
            rospy.signal_shutdown("subscribe_topics: Invalid camera type")
            return

        if not self.rgb_topic or not self.depth_topic:
            rospy.logerr("subscribe_topics: Camera topic not found.")
            rospy.signal_shutdown("subscribe_topics: Camera topic not found")
            return

        self.image_sub = rospy.Subscriber(self.rgb_topic, Image, self.image_callback)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback)

    def image_callback(self, msg):
        """Callback function to process RGB images from the subscribed topic."""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            print(cv_image.shape)

            # Start timing when the first frame is received
            if self.start_time is None:
                self.start_time = time.time()
                self.image_save_time = self.start_time

            # Store frames for video saving if enabled
            if self.config.get("save_video", False):
                self.rgb_frames.append(cv_image)

                # Check if the video duration has been reached
                elapsed_time = time.time() - self.start_time
                if elapsed_time >= self.video_duration:
                    video_path = os.path.join(self._face_detection_test_package_path, 'data', f'captured_rgb_video_{int(self.start_time)}.mp4')
                    self.save_video(self.rgb_frames, video_path)

                    # Reset for the next video capture
                    self.rgb_frames = []
                    self.start_time = None

            # Save individual RGB images at specified intervals
            current_time = time.time()
            if self.config.get("save_image", False) and (current_time - self.image_save_time >= self.image_interval):
                image_path = os.path.join(self._face_detection_test_package_path, 'data', f'captured_rgb_image_{int(current_time)}.png')
                self.save_image(cv_image, image_path)
                self.image_save_time = current_time

        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def depth_callback(self, msg):
        """Callback function to process Depth images from the subscribed topic."""
        try:
            # Convert ROS Image message to OpenCV format
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Store depth frames for video saving if enabled
            if self.config.get("save_video", False):
                self.depth_frames.append(cv_depth)

                # Check if the video duration has been reached
                elapsed_time = time.time() - self.start_time
                if elapsed_time >= self.video_duration:
                    video_path = os.path.join(self._face_detection_test_package_path, 'data', f'captured_depth_video_{int(self.start_time)}.mp4')
                    self.save_video(self.depth_frames, video_path, is_depth=True)

                    # Reset for the next video capture
                    self.depth_frames = []

            # Save individual Depth images at specified intervals
            current_time = time.time()
            if self.config.get("save_image", False) and (current_time - self.image_save_time >= self.image_interval):
                image_path = os.path.join(self._face_detection_test_package_path, 'data', f'captured_depth_image_{int(current_time)}.png')
                self.save_image(cv_depth, image_path, is_depth=True)

        except Exception as e:
            rospy.logerr(f"Error in depth_callback: {e}")

    def save_video(self, frames, output_path, is_depth=False):
        """Save video frames as an MP4 file.
        
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
                    frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                video_writer.write(frame)

            video_writer.release()
            rospy.loginfo(f"Video saved successfully at: {output_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save video: {e}")

    def save_image(self, image, output_path, is_depth=False):
        """Save an image as a PNG file.
        
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
                image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

            cv2.imwrite(output_path, image)
            rospy.loginfo(f"Image saved successfully at: {output_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save image: {e}")

    def test_media_pipe(self):
        """Perform MediaPipe face detection test."""
        rospy.loginfo("Starting MediaPipe face detection test...")
        # Add your test logic here
        # Example: rospy.loginfo(f"Using configuration: {self.config['mediapipe']}")

    def test_sixdrep(self):
        """Perform Sixdrep face detection test."""
        rospy.loginfo("Starting Sixdrep face detection test...")
        # Add your test logic here
        # Example: rospy.loginfo(f"Using configuration: {self.config['sixdrep']}")

    def run_tests(self):
        """Run both tests in sequence."""
        self.test_media_pipe()
        self.test_sixdrep()

        # Save video if enabled in the configuration
        if self.config.get("save_video", False):
            rgb_video_path = os.path.join(self._face_detection_test_package_path, 'output', 'captured_rgb_video.mp4')
            depth_video_path = os.path.join(self._face_detection_test_package_path, 'output', 'captured_depth_video.mp4')
            self.save_video(self.rgb_frames, rgb_video_path)
            self.save_video(self.depth_frames, depth_video_path, is_depth=True)
