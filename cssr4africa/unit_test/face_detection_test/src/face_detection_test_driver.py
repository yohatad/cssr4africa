#!/usr/bin/env python3

import rospy
import cv2
import json
import os
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImagePublisherNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('video_publisher', anonymous=True)

        # Use rospkg to get the package directory
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("face_detection_test")  # Adjust package name if necessary

        # Construct config file path
        config_path = os.path.join(package_path, "config", "face_detection_test_configuration.json")

        # Load JSON configuration
        self.config = self.load_config(config_path)

        # Get the video number from the config
        video_number = str(self.config.get("video_file", "1"))  # Default to "1"
        self.loop_video = self.config.get("loop_video", False)  # Default is False

        # Construct file names automatically
        color_video_file = f"color_{video_number}.mp4"
        depth_video_file = f"depth_{video_number}.mp4"

        # Construct paths
        color_video_path = os.path.join(package_path, "data", color_video_file)
        depth_video_path = os.path.join(package_path, "data", depth_video_file)

        # Validate files exist
        if not os.path.exists(color_video_path):
            rospy.logerr(f"Color video file {color_video_path} not found!")
            raise FileNotFoundError(f"Color video file {color_video_path} not found!")

        if not os.path.exists(depth_video_path):
            rospy.logerr(f"Depth video file {depth_video_path} not found!")
            raise FileNotFoundError(f"Depth video file {depth_video_path} not found!")

        rospy.loginfo(f"Using color video: {color_video_path} | Depth video: {depth_video_path} | Loop: {self.loop_video}")

        # Open video files
        self.color_video_cap = cv2.VideoCapture(color_video_path)
        self.depth_video_cap = cv2.VideoCapture(depth_video_path)

        # Publishers for color and depth images
        self.color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/camera/aligned_depth_to_color/image_raw', Image, queue_size=10)
        self.bridge = CvBridge()

    def load_config(self, config_path):
        """ Load configuration from JSON file """
        try:
            with open(config_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            rospy.logerr(f"Failed to load JSON config: {e}")
            raise

    def publish_images(self):
        """ Publish color and depth images from video files """

        # Read frames
        ret_color, frame_color = self.color_video_cap.read()
        ret_depth, frame_depth = self.depth_video_cap.read()

        # Handle end of video
        if not ret_color or not ret_depth:
            if self.loop_video:
                rospy.logwarn("End of video streams. Restarting...")
                self.color_video_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.depth_video_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                return
            else:
                rospy.logwarn("Videos ended. Stopping publisher.")
                rospy.signal_shutdown("Video playback completed.")
                return

        # Convert depth image to 16-bit format if needed
        if len(frame_depth.shape) == 3:  # Convert if depth is in 3-channel format
            frame_depth = cv2.cvtColor(frame_depth, cv2.COLOR_BGR2GRAY)
        frame_depth = frame_depth.astype("uint16")  # Ensure it's in 16-bit format

        # Convert to ROS Image messages
        color_msg = self.bridge.cv2_to_imgmsg(frame_color, encoding="bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(frame_depth, encoding="16UC1")

        # Set timestamps and frames
        current_time = rospy.Time.now()
        color_msg.header.stamp = current_time
        depth_msg.header.stamp = current_time

        color_msg.header.frame_id = "camera_color_optical_frame"
        depth_msg.header.frame_id = "camera_depth_optical_frame"

        # Publish images
        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)
        rospy.loginfo("Published color and depth frames.")

    def run(self):
        """ Main loop to publish images """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.publish_images()
            rate.sleep()

if __name__ == "__main__":
    try:
        node = ImagePublisherNode()
        node.run()
    except rospy.ROSInterruptException:
        pass