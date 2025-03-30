#!/usr/bin/env python3
"""
ros_image_saver.py - Save RGB and depth images from ROS topics as numpy arrays and PNG files.

This node subscribes to synchronized RGB and depth camera topics, then saves them as:
1. RGB image as PNG (for reference/visualization)
2. RGB image as NumPy array (.npy)
3. Depth image as NumPy array (.npy)

Images are saved at a configurable interval (default: 5 seconds).
"""

import rospy
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import time

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)
        
        # Get parameters
        self.save_dir = '/home/yoha/yohannes/data4/'
        self.save_interval = rospy.get_param('~save_interval', 5.0)  # seconds
        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/color/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/aligned_depth_to_color/image_raw')
        
        # Create save directory if it doesn't exist
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            rospy.loginfo(f"Created directory: {self.save_dir}")
        
        # Initialize bridge
        self.bridge = CvBridge()
        
        # Last save timestamp
        self.last_save_time = time.time()
        
        # Set up synchronized subscribers
        self.rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        
        # Time synchronizer for the two streams
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self.callback)
        
        rospy.loginfo(f"Image saver initialized. Saving to {self.save_dir} every {self.save_interval} seconds.")
        rospy.loginfo(f"RGB topic: {self.rgb_topic}, Depth topic: {self.depth_topic}")
    
    def callback(self, rgb_msg, depth_msg):
        # Check if it's time to save a new image
        current_time = time.time()
        if current_time - self.last_save_time < self.save_interval:
            return
        
        # Update last save time
        self.last_save_time = current_time
        
        try:
            # Convert ROS Image messages to OpenCV format
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            
            # For depth image, preserve the original encoding
            encoding = depth_msg.encoding
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, encoding)
            
            # Generate filename with timestamp
            timestamp = rospy.Time.now().to_sec()
            filename_base = os.path.join(self.save_dir, f"{timestamp:.6f}")
            
            # Save RGB image as PNG (for reference)
            cv2.imwrite(f"{filename_base}_rgb.png", rgb_image)
            
            # Save RGB as NumPy array
            np.save(f"{filename_base}_rgb.npy", rgb_image)
            
            # Save depth as NumPy array
            np.save(f"{filename_base}_depth.npy", depth_image)
            
            rospy.loginfo(f"Saved images with timestamp: {timestamp:.6f}")
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Error saving images: {e}")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        saver = ImageSaver()
        saver.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Image saver node interrupted.")