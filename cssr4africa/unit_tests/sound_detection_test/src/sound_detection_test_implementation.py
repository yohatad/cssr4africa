"""
sound_detection_test_implementation.py Implementation code for running the Sound Detection and Processing unit test.

Author: Yohannes Tadesse Haile
Date: April 10, 2025
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
import time
import json
import numpy as np
import soundfile as sf
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray, Float32
from unit_tests.msg import microphone_test_msg_file
from datetime import datetime
from threading import Lock

class SoundDetectionTest:
    """
    SoundDetectionTest records and analyzes audio data for testing the sound detection system.
    It can record both filtered and unfiltered audio and generate plots for analysis.
    """
    def __init__(self):
        """
        Initialize the SoundDetectionTest class.
        Sets up configuration, ROS subscribers, and data structures for audio analysis.
        """
        # Set node name for consistent logging
        self.node_name = "soundDetectionTest"
        
        self.rospack = rospkg.RosPack()
        try:
            self.unit_test_package_path = self.rospack.get_path('unit_tests')
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"{self.node_name}: ROS package not found: {e}")
            raise RuntimeError(f"Required ROS package not found: {e}")

        # Read configuration
        self.config = self.read_json_file()
        if not self.config:
            rospy.logerr(f"{self.node_name}: Failed to load configuration. Exiting.")
            raise RuntimeError("Configuration file could not be loaded.")
        
        # Set up configuration parameters with defaults
        self.sample_rate = 48000
        self.save_dir = self.unit_test_package_path + '/sound_detection_test/data/'
        self.record_filtered = self.config.get("recordFiltered", True)
        self.record_unfiltered = self.config.get("recordUnfiltered", True)
        self.generate_plots = self.config.get("generatePlots", True)
        self.record_duration = self.config.get("recordDuration", 10)
        self.plot_interval = self.config.get("plotInterval", 10)
        self.verbose_mode = self.config.get("verboseMode", False)
        self.plot_dpi = self.config.get("plotDpi", 150)
        self.max_direction_points = self.config.get("maxDirectionPoints", 100)
        self.direction_plot_ylimit = self.config.get("directionPlotYlimit", 90)
        
        # Create save directory if it doesn't exist
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            rospy.loginfo(f"{self.node_name}: Created directory for audio recordings: {self.save_dir}")


        # Timer for periodic status message
        self.status_timer = rospy.get_time()
        
        # Locks for thread safety
        self.filtered_lock = Lock()
        self.unfiltered_lock = Lock()
        self.direction_lock = Lock()
        
        # State variables for filtered audio
        self.filtered_audio_buffer = []
        self.is_recording_filtered = False
        self.filtered_recording_start_time = None
        
        # State variables for unfiltered audio
        self.unfiltered_audio_buffer = []
        self.is_recording_unfiltered = False
        self.unfiltered_recording_start_time = None
        
        # Plotting buffer for 10 seconds of audio (10 sec * sample_rate)
        self.plot_buffer_size = self.sample_rate * self.plot_interval
        self.filtered_plot_buffer = np.zeros(self.plot_buffer_size, dtype=np.float32)
        self.unfiltered_plot_buffer = np.zeros(self.plot_buffer_size, dtype=np.float32)
        self.filtered_plot_index = 0
        self.unfiltered_plot_index = 0
        
        # Direction data for plotting
        self.direction_data = []
        self.direction_timestamps = []
        self.last_plot_time = time.time()
        
        # Get the original microphone topic from the config
        self.microphone_topic = self.extract_topics('Microphone')
        if not self.microphone_topic:
            rospy.logwarn(f"{self.node_name}: Microphone topic not found. Will only record filtered audio.")
            self.record_unfiltered = False
        
        # Subscribe to the filtered audio signal topic
        if self.record_filtered:
            self.filtered_sub = rospy.Subscriber("soundDetection/signal", Float32MultiArray, self.filtered_audio_callback)
            rospy.loginfo(f"{self.node_name}: Subscribed to filtered audio: soundDetection/signal")
        
        # Subscribe to the original microphone topic for unfiltered audio
        if self.record_unfiltered and self.microphone_topic:
            self.unfiltered_sub = rospy.Subscriber(self.microphone_topic, microphone_test_msg_file, self.unfiltered_audio_callback)
            rospy.loginfo(f"{self.node_name}: Subscribed to unfiltered audio: {self.microphone_topic}")
        
        # Subscribe to the direction topic
        self.direction_sub = rospy.Subscriber("soundDetection/direction", Float32, self.direction_callback)
        rospy.loginfo(f"{self.node_name}: Subscribed to direction data: soundDetection/direction")
        
        # Set a timer to periodically generate plots if enabled
        if self.generate_plots:
            rospy.Timer(rospy.Duration(self.plot_interval), self.plot_callback)
        
        rospy.loginfo(f"{self.node_name}: Initialized successfully")
        rospy.loginfo(f"{self.node_name}: Recording will automatically stop after {self.record_duration} seconds")
        
        # Register shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

    def read_json_file(self):
        """
        Read the configuration file and return the parsed JSON object.
        
        Returns:
            dict: Configuration dictionary or None if file couldn't be read.
        """
        try:
            config_path = os.path.join(self.unit_test_package_path, 'sound_detection_test/config', 'sound_detection_test_configuration.json')
            if not os.path.exists(config_path):
                rospy.logerr(f"{self.node_name}: read_json_file: Configuration file not found at {config_path}")
                return None
                
            with open(config_path, 'r') as file:
                config = json.load(file)
                return config
                
        except json.JSONDecodeError as e:
            rospy.logerr(f"{self.node_name}: read_json_file: Error decoding JSON file: {e}")
            return None
        except Exception as e:
            rospy.logerr(f"{self.node_name}: read_json_file: Unexpected error: {e}")
            return None

    def extract_topics(self, topic_key):
        """
        Extract the topic name for a given key from the topics data file.
        
        Args:
            topic_key (str): Key to search for in the topics file.
            
        Returns:
            str: The topic name or None if not found.
        """
        try:
            config_path = os.path.join(self.unit_test_package_path, 'sound_detection_test/data', 'pepper_topics.dat')
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    for line in file:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            continue
                        key, value = line.split(maxsplit=1)
                        if key.lower() == topic_key.lower():
                            return value
            else:
                rospy.logerr(f"{self.node_name}: Topics data file not found at {config_path}")
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"{self.node_name}: ROS package not found: {e}")
        return None
    
    def filtered_audio_callback(self, msg):
        """
        Process incoming filtered audio data from soundDetection/signal.
        
        The data is in Float32MultiArray format, with values in the range [-1.0, 1.0].
        
        Args:
            msg (Float32MultiArray): The audio data message
        """
        if not self.record_filtered:
            return
            
        # Convert message data to numpy array
        audio_data = np.array(msg.data, dtype=np.float32)
        
        with self.filtered_lock:
            # If we're currently recording, add to buffer
            if self.is_recording_filtered:
                self.filtered_audio_buffer.extend(audio_data.tolist())
            else:
                # Start a new recording
                self.is_recording_filtered = True
                self.filtered_recording_start_time = rospy.get_time()
                self.filtered_audio_buffer = audio_data.tolist()
                rospy.loginfo(f"{self.node_name}: Started new FILTERED audio recording")
            
            # Check if we've reached the recording duration
            buffer_time = rospy.get_time() - self.filtered_recording_start_time
            
            if buffer_time >= self.record_duration:
                self.save_filtered_audio()
    
    def unfiltered_audio_callback(self, msg):
        """
        Process incoming unfiltered audio data from the original microphone topic.
        
        The data is in microphone_test_msg_file message type with frontLeft and frontRight arrays.
        
        Args:
            msg (microphone_test_msg_file): The audio data message
        """
        if not self.record_unfiltered:
            return
            
        try:
            # Print a status messave every 10 seconds
            if rospy.get_time() - self.status_timer > 10:
                rospy.loginfo(f"{self.node_name}: running")
                self.status_timer = rospy.get_time()

            # Extract only the left channel data from int16 array and normalize to float
            audio_data_int16 = np.array(msg.frontLeft, dtype=np.int16)
            audio_data = audio_data_int16.astype(np.float32) / 32767.0
            
            with self.unfiltered_lock:
                # If we're currently recording, add to buffer
                if self.is_recording_unfiltered:
                    self.unfiltered_audio_buffer.extend(audio_data.tolist())
                else:
                    # Start a new recording
                    self.is_recording_unfiltered = True
                    self.unfiltered_recording_start_time = rospy.get_time()
                    self.unfiltered_audio_buffer = audio_data.tolist()
                    rospy.loginfo(f"{self.node_name}: Started new UNFILTERED audio recording")
                
                # Check if we've reached the recording duration
                buffer_time = rospy.get_time() - self.unfiltered_recording_start_time
                
                if buffer_time >= self.record_duration:
                    self.save_unfiltered_audio()
                    
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error in unfiltered audio callback: {e}")
    
    def direction_callback(self, msg):
        """
        Process incoming direction data from soundDetection/direction.
        
        Args:
            msg (Float32): The direction angle in degrees
        """
        with self.direction_lock:
            current_time = time.time() - self.last_plot_time
            self.direction_data.append(msg.data)
            self.direction_timestamps.append(current_time)
    
    def plot_callback(self, event):
        """
        Periodically called to generate plots from the collected data.
        
        Args:
            event (rospy.timer.TimerEvent): Timer event object
        """
        if not self.generate_plots:
            return
            
        try:
            # Plot direction data
            self.plot_direction_data()
            
            # Reset direction data
            with self.direction_lock:
                self.direction_data = []
                self.direction_timestamps = []
                self.last_plot_time = time.time()
                
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error in plot_callback: {e}")
    
    
    def plot_direction_data(self):
        """
        Plot the sound direction data over time.
        """
        with self.direction_lock:
            # Check if we have direction data to plot
            if not self.direction_data:
                if self.verbose_mode:
                    rospy.loginfo(f"{self.node_name}: No direction data to plot")
                return
            
            # Limit the number of data points if there are too many
            if len(self.direction_data) > self.max_direction_points:
                # Take evenly spaced samples
                indices = np.linspace(0, len(self.direction_data)-1, self.max_direction_points, dtype=int)
                direction_data = [self.direction_data[i] for i in indices]
                timestamp_data = [self.direction_timestamps[i] for i in indices]
            else:
                direction_data = self.direction_data
                timestamp_data = self.direction_timestamps
            
            # Make a copy of the data
            directions = np.array(direction_data)
            timestamps = np.array(timestamp_data)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        plt.figure(figsize=(10, 6))
        plt.plot(timestamps, directions, 'bo-', markersize=4)
        plt.title('Sound Direction Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Direction Angle (degrees)')
        plt.grid(True)
        plt.ylim(-self.direction_plot_ylimit, self.direction_plot_ylimit)  # Range from config
        
        # Add horizontal lines at 0, -45, and 45 degrees
        plt.axhline(y=0, color='r', linestyle='-', alpha=0.3)
        plt.axhline(y=-45, color='g', linestyle='--', alpha=0.3)
        plt.axhline(y=45, color='g', linestyle='--', alpha=0.3)
        
        plt.tight_layout()
        
        # Save the plot
        plot_path = os.path.join(self.save_dir, f'sound_detection_test_direction_data_{timestamp}.png')
        plt.savefig(plot_path)
        plt.close()
        
        if self.verbose_mode:
            rospy.loginfo(f"{self.node_name}: Direction data plot saved to: {plot_path}")
    
    def save_filtered_audio(self):
        """Save the current filtered audio buffer as a WAV file."""
        if not self.filtered_audio_buffer:
            rospy.logwarn(f"{self.node_name}: Cannot save empty filtered audio buffer")
            return
        
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Save as WAV file
            wav_filepath = os.path.join(self.save_dir, f"sound_detection_test_filtered_{timestamp}.wav")
            
            # Convert audio buffer to numpy array
            audio_np = np.array(self.filtered_audio_buffer, dtype=np.float32)
            
            # Save as WAV file
            sf.write(wav_filepath, audio_np, self.sample_rate, format='WAV')
            
            # Log success
            duration = len(audio_np) / self.sample_rate  # Duration in seconds
            if self.verbose_mode:
                rospy.loginfo(f"{self.node_name}: Saved {duration:.2f}s FILTERED audio to {wav_filepath}")
            
            # Reset state for next recording
            self.filtered_audio_buffer = []
            self.is_recording_filtered = False
            
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error saving filtered audio: {e}")
    
    def save_unfiltered_audio(self):
        """Save the current unfiltered audio buffer as a mono WAV file."""
        if not self.unfiltered_audio_buffer:
            rospy.logwarn(f"{self.node_name}: Cannot save empty unfiltered audio buffer")
            return
            
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Save as WAV file
            wav_filepath = os.path.join(self.save_dir, f"sound_detection_test_unfiltered_{timestamp}.wav")
            
            # Convert audio buffer to numpy array
            audio_np = np.array(self.unfiltered_audio_buffer, dtype=np.float32)
            
            # Save as WAV file (mono)
            sf.write(wav_filepath, audio_np, self.sample_rate, format='WAV')
            
            # Log success
            duration = len(audio_np) / self.sample_rate  # Duration in seconds
            if self.verbose_mode:
                rospy.loginfo(f"{self.node_name}: Saved {duration:.2f}s UNFILTERED mono audio to {wav_filepath}")
            
            # Reset state for next recording
            self.unfiltered_audio_buffer = []
            self.is_recording_unfiltered = False
            
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error saving unfiltered audio: {e}")
    
    def shutdown_hook(self):
        """
        Clean up resources when the node is shutting down.
        """
        if self.verbose_mode:
            rospy.loginfo(f"{self.node_name}: Shutting down")