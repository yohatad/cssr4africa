import os
import time
import numpy as np
import soundfile as sf
import rospkg
import json
import rospy
from std_msgs.msg import Float32MultiArray
from sound_detection.msg import sound_detection
from datetime import datetime

class SoundDetectionTest:
    def __init__(self):
        """
        Initialize the SoundDetectionTest class.
        
        This sets up ROS subscribers to both filtered and unfiltered audio streams,
        and saves the received audio data as mp3 files.
        """
        # Manually set parameters
        self.save_dir = '/home/yoha/workspace/pepper_rob_ws/src/cssr4africa/unit_test/sound_detection_test/data'
        self.sample_rate = 48000
        self.record_filtered = True
        self.record_unfiltered = True
        
        # Create save directory if it doesn't exist
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            rospy.loginfo(f"Created directory for audio recordings: {self.save_dir}")
        
        # State variables for filtered audio
        self.filtered_audio_buffer = []
        self.is_recording_filtered = False
        self.filtered_recording_start_time = None
        
        # State variables for unfiltered audio
        self.unfiltered_audio_buffer = []
        self.is_recording_unfiltered = False
        self.unfiltered_recording_start_time = None
        
        # Get the original microphone topic from the config
        self.microphone_topic = self.extract_topics('Microphone')
        if not self.microphone_topic:
            rospy.logwarn("Microphone topic not found. Will only record filtered audio.")
            self.record_unfiltered = False
        
        # Subscribe to the filtered audio signal topic
        if self.record_filtered:
            self.filtered_sub = rospy.Subscriber("soundDetection/signal", Float32MultiArray, self.filtered_audio_callback)
            rospy.loginfo("Subscribed to filtered audio: soundDetection/signal")
        
        # Subscribe to the original microphone topic for unfiltered audio
        if self.record_unfiltered and self.microphone_topic:
            self.unfiltered_sub = rospy.Subscriber(self.microphone_topic, sound_detection, self.unfiltered_audio_callback)
            rospy.loginfo(f"Subscribed to unfiltered audio: {self.microphone_topic}")
        
        rospy.loginfo("SoundDetectionTest: Initialized successfully")
        rospy.loginfo("Recording will automatically stop after 10 seconds")
    

    @staticmethod
    def extract_topics(topic_key):
        """
        Extract the topic name for a given key from the topics data file.
        Reused from sound_implementation.py
        """
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('sound_detection')
            config_path = os.path.join(package_path, 'data', 'pepper_topics.dat')
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
                rospy.logerr(f"Topics data file not found at {config_path}")
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"ROS package 'sound_detection' not found: {e}")
        return None
    
    def filtered_audio_callback(self, msg):
        """
        Process incoming filtered audio data from soundDetection/signal.
        
        The data is in Float32MultiArray format, with values in the range [-1.0, 1.0].
        """
        if not self.record_filtered:
            return
            
        # Convert message data to numpy array
        audio_data = np.array(msg.data, dtype=np.float32)
        
        # If we're currently recording, add to buffer
        if self.is_recording_filtered:
            self.filtered_audio_buffer.extend(audio_data.tolist())
        else:
            # Start a new recording
            self.is_recording_filtered = True
            self.filtered_recording_start_time = rospy.get_time()
            self.filtered_audio_buffer = audio_data.tolist()
            rospy.loginfo("Started new FILTERED audio recording")
        
        # Check if we've reached the 10-second recording duration
        buffer_time = rospy.get_time() - self.filtered_recording_start_time
        
        if buffer_time >= 10.0:
            self.save_filtered_audio()
    
    def unfiltered_audio_callback(self, msg):
        """
        Process incoming unfiltered audio data from the original microphone topic.
        
        The data is in sound_detection message type with frontLeft and frontRight arrays.
        """
        if not self.record_unfiltered:
            return
            
        try:
            # Extract only the left channel data from int16 array and normalize to float
            # We're only using frontLeft as requested since the audio is mono
            audio_data_int16 = np.array(msg.frontLeft, dtype=np.int16)
            audio_data = audio_data_int16.astype(np.float32) / 32767.0
            
            # If we're currently recording, add to buffer
            if self.is_recording_unfiltered:
                self.unfiltered_audio_buffer.extend(audio_data.tolist())
            else:
                # Start a new recording
                self.is_recording_unfiltered = True
                self.unfiltered_recording_start_time = rospy.get_time()
                self.unfiltered_audio_buffer = audio_data.tolist()
                rospy.loginfo("Started new UNFILTERED audio recording")
            
            # Check if we've reached the 10-second recording duration
            buffer_time = rospy.get_time() - self.unfiltered_recording_start_time
            
            if buffer_time >= 10.0:
                self.save_unfiltered_audio()
                
        except Exception as e:
            rospy.logerr(f"Error in unfiltered audio callback: {e}")
    
    def save_filtered_audio(self):
        """Save the current filtered audio buffer as a WAV file"""
        if not self.filtered_audio_buffer:
            rospy.logwarn("Cannot save empty filtered audio buffer")
            return
        
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Save as WAV file
            wav_filepath = os.path.join(self.save_dir, f"filtered_{timestamp}.wav")
            
            # Convert audio buffer to numpy array
            audio_np = np.array(self.filtered_audio_buffer, dtype=np.float32)
            
            # Save as WAV file
            sf.write(wav_filepath, audio_np, self.sample_rate, format='WAV')
            
            # Log success
            duration = len(audio_np) / self.sample_rate  # Duration in seconds
            rospy.loginfo(f"Saved {duration:.2f}s FILTERED audio to {wav_filepath}")
            
            # Reset state for next recording
            self.filtered_audio_buffer = []
            self.is_recording_filtered = False
            
        except Exception as e:
            rospy.logerr(f"Error saving filtered audio: {e}")
    
    def save_unfiltered_audio(self):
        """Save the current unfiltered audio buffer as a mono WAV file"""
        if not self.unfiltered_audio_buffer:
            rospy.logwarn("Cannot save empty unfiltered audio buffer")
            return
            
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Save as WAV file
            wav_filepath = os.path.join(self.save_dir, f"unfiltered_{timestamp}.wav")
            
            # Convert audio buffer to numpy array
            audio_np = np.array(self.unfiltered_audio_buffer, dtype=np.float32)
            
            # Save as WAV file (mono)
            sf.write(wav_filepath, audio_np, self.sample_rate, format='WAV')
            
            # Log success
            duration = len(audio_np) / self.sample_rate  # Duration in seconds
            rospy.loginfo(f"Saved {duration:.2f}s UNFILTERED mono audio to {wav_filepath}")
            
            # Reset state for next recording
            self.unfiltered_audio_buffer = []
            self.is_recording_unfiltered = False
            
        except Exception as e:
            rospy.logerr(f"Error saving unfiltered audio: {e}")
    
    def save_raw_audio(self, audio_data_int16):
        """Optional method to save int16 audio data directly without conversion to float"""
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Save as WAV
            wav_filepath = os.path.join(self.save_dir, f"raw_{timestamp}.wav")
            
            # Save as WAV file (mono, int16)
            sf.write(wav_filepath, audio_data_int16, self.sample_rate, format='WAV', subtype='PCM_16')
            
            # Log success
            duration = len(audio_data_int16) / self.sample_rate  # Duration in seconds
            rospy.loginfo(f"Saved {duration:.2f}s RAW int16 audio to {wav_filepath}")
            
        except Exception as e:
            rospy.logerr(f"Error saving raw audio: {e}")
            
    def convert_float_to_int16(self, float_data):
        """Convert float audio in [-1, 1] range to int16"""
        # Ensure the data is within valid range
        float_data = np.clip(float_data, -1.0, 1.0)
        # Convert to int16
        return (float_data * 32767).astype(np.int16)

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('sound_detection_test', anonymous=True)
    
    # Create and initialize the SoundDetectionTest instance
    test_node = SoundDetectionTest()
    
    # Keep the node running
    rospy.spin()