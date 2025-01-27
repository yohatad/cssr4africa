#!/usr/bin/env python3

import rospy
import cv2
import subprocess
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImagePublisherNode:
    def __init__(self, color_image_path, depth_image_path):
        # Initialize ROS node
        rospy.init_node('image_publisher', anonymous=True)

        # Publishers for color and depth images
        self.color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/camera/aligned_depth_to_color/image_raw', Image, queue_size=10)

        # Create a CvBridge instance
        self.bridge = CvBridge()

        # Load the images
        self.color_image = cv2.imread(color_image_path, cv2.IMREAD_COLOR)  # Load color image as BGR
        self.depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)  # Load depth image as 16-bit

        if self.color_image is None:
            rospy.logerr(f"Failed to load color image from {color_image_path}")
            raise FileNotFoundError(f"Color image not found at {color_image_path}")
        if self.depth_image is None:
            rospy.logerr(f"Failed to load depth image from {depth_image_path}")
            raise FileNotFoundError(f"Depth image not found at {depth_image_path}")

    def publish_images(self):
        # Get the current ROS time
        current_time = rospy.Time.now()

        # Publish the images as ROS messages
        rospy.loginfo("Publishing images...")

        # Create Image messages
        color_msg = self.bridge.cv2_to_imgmsg(self.color_image, encoding="bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(self.depth_image, encoding="16UC1")

        # Set the timestamp
        color_msg.header.stamp = current_time
        depth_msg.header.stamp = current_time

        color_msg.header.frame_id = "camera_color_optical_frame"
        depth_msg.header.frame_id = "camera_depth_optical_frame"

        # Publish the messages
        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)

        rospy.loginfo("Images published successfully.")

        # Shut down the node
        #rospy.signal_shutdown("Images published successfully. Node shutting down.")

if __name__ == "__main__":
    # Define the paths to your saved color and depth images
    color_image_path = "/root/workspace/pepper_rob_ws/images/color_20241127_071120.png"  
    depth_image_path = "/root/workspace/pepper_rob_ws/images/depth_20241127_071120.png"  

    try:
        node = ImagePublisherNode(color_image_path, depth_image_path)

        rospy.sleep(2)

        times_to_publish = 8
        # Publish the single pair of images multiple times
        for _ in range(times_to_publish):  # Publish 10 times to simulate a short stream
            node.publish_images()
            rospy.sleep(1)  

        rospy.loginfo(f"Published single pair of images {times_to_publish} times. Shutting down.")

        rospy.sleep(2)

        # Signal shutdown for the current node
        rospy.signal_shutdown("Unit test completed. Shutting down skeletal_model_stub.")
        

    except rospy.ROSInterruptException:
        pass
