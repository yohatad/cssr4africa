#!/usr/bin/env python3

import rospy
import numpy as np
import soundfile as sf
from datetime import datetime
import noisereduce as nr
from cssr_system.msg import microphone_msg_file
from std_msgs.msg import Float32MultiArray
import os

class ChunkRecorder:
    """
    Records audio from the microphone topic with 4096-sample blocks,
    applies noise reduction using a 2-second context window and a custom noise profile,
    and saves it to the current folder.
    """

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('chunk_recorder', anonymous=True)
        
        # Configuration parameters
        self.frequency_sample = 48000      # Sampling frequency (Hz)
        self.record_duration = 10.0        # Total recording duration in seconds
        self.context_duration = 2.0        # Context window duration in seconds
        self.block_size = 4096             # Microphone block size (must match incoming data)
        
        # Calculate sizes in samples
        self.total_samples = int(self.frequency_sample * self.record_duration)
        self.context_size = int(self.frequency_sample * self.context_duration)
        self.total_blocks = int(np.ceil(self.total_samples / self.block_size))
        
        # Buffer to store raw audio data
        self.raw_buffer = []
        self.blocks_received = 0
        self.is_recording = True
        
        # Buffer to store processed audio data
        self.processed_buffer = []
        
        # Context window for processing
        self.context_window = np.zeros(self.context_size, dtype=np.float32)

        
        # Get microphone topic - using similar approach as in SoundDetectionNode
        microphone_topic = self.get_microphone_topic()
        if not microphone_topic:
            rospy.logerr("Microphone topic not found. Using default /pepper_robot/audio/microphones")
            microphone_topic = "/pepper_robot/audio/microphones"
        
        # Status reporting
        rospy.loginfo(f"Chunk Recorder: Started - Will record {self.record_duration} seconds of audio")
        rospy.loginfo(f"Chunk Recorder: Using {self.context_duration} second context window")
        rospy.loginfo(f"Chunk Recorder: Processing block size: {self.block_size} samples ({self.block_size/self.frequency_sample:.3f} seconds)")
        rospy.loginfo(f"Chunk Recorder: Total blocks to collect: {self.total_blocks}")
        rospy.loginfo(f"Chunk Recorder: Subscribing to topic {microphone_topic}")
        
        # Subscribe to the microphone topic
        self.audio_sub = rospy.Subscriber(microphone_topic, microphone_msg_file, self.audio_callback)
        
    def get_microphone_topic(self):
        """
        Get the microphone topic from the ROS parameter server or use a fallback method.
        
        Returns:
            str: Microphone topic name
        """
        # First try to get from parameter server
        mic_topic = rospy.get_param('/microphone_topic', None)
        if mic_topic:
            return mic_topic
            
        # If not found, try to extract from the same source as SoundDetectionNode
        try:
            from rospkg import RosPack
            rospack = RosPack()
            package_path = rospack.get_path('cssr_system')
            topics_file = os.path.join(package_path, 'sound_detection/data', 'pepper_topics.dat')
            
            if os.path.exists(topics_file):
                with open(topics_file, 'r') as file:
                    for line in file:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            continue
                        key, value = line.split(maxsplit=1)
                        if key.lower() == 'microphone':
                            return value
        except Exception as e:
            rospy.logwarn(f"Error getting microphone topic: {e}")
        
        return None
    
    def audio_callback(self, msg):
        """
        Process incoming audio data from the microphone.
        
        Args:
            msg (microphone_msg_file): The audio data message
        """
        if not self.is_recording:
            return
            
        try:
            # Convert int16 data to float32 and normalize to [-1.0, 1.0]
            # Using front left channel only
            sigIn_frontLeft = np.array(msg.frontLeft, dtype=np.float32) / 32767.0
            
            # Verify that the incoming data size matches our expected block size
            if len(sigIn_frontLeft) != self.block_size:
                rospy.logwarn(f"Unexpected block size: {len(sigIn_frontLeft)}. Expected: {self.block_size}")
            
            # Add to raw buffer
            self.raw_buffer.extend(sigIn_frontLeft)
            self.blocks_received += 1
            
            # Report progress every 5 blocks
            if self.blocks_received % 5 == 0 or self.blocks_received == 1:
                percentage = min(self.blocks_received / self.total_blocks * 100, 100)
                seconds = min(self.blocks_received * self.block_size / self.frequency_sample, self.record_duration)
                rospy.loginfo(f"Recording progress: {percentage:.1f}% ({seconds:.1f}/{self.record_duration} seconds)")
            
            # Update context window and process the new block
            self.process_block(sigIn_frontLeft)
            
            # Check if we've collected enough blocks
            if self.blocks_received >= self.total_blocks:
                rospy.loginfo("Recording complete. Saving audio...")
                self.is_recording = False
                self.save_audio()
                
        except Exception as e:
            rospy.logerr(f"Error in audio_callback: {e}")
    
    def process_block(self, current_block):
        """
        Process a single block of audio using the context window and custom noise profile.
        
        Args:
            current_block (np.ndarray): New audio block to process
        """
        try:
            # Update context window: shift old data left and add new block at the end
            block_size_actual = len(current_block)
            self.context_window = np.roll(self.context_window, -block_size_actual)
            self.context_window[-block_size_actual:] = current_block
            
       
            # Fallback to stationary noise reduction if no profile is available
            reduced_context = nr.reduce_noise(
                y=self.context_window,
                sr=self.frequency_sample,
                stationary=True,
                prop_decrease=0.5
            )
        
            # Extract only the most recent block from the processed context
            processed_block = reduced_context[-block_size_actual:]
            
            # Apply gain
            gain = 5.5  # You can adjust this value
            processed_block = processed_block * gain
            
            # Prevent clipping
            max_val = np.max(np.abs(processed_block))
            if max_val > 1.0:
                processed_block = processed_block / max_val
            
            # Add to processed buffer
            self.processed_buffer.extend(processed_block)
            
        except Exception as e:
            rospy.logerr(f"Error processing block: {e}")
    
    def save_audio(self):
        """
        Save the raw and processed audio to files.
        """
        try:
            # Convert buffers to numpy arrays
            # Limit to total_samples to ensure consistent file lengths
            raw_audio = np.array(self.raw_buffer[:self.total_samples], dtype=np.float32)
            processed_audio = np.array(self.processed_buffer[:self.total_samples], dtype=np.float32)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            processed_filename = f"fan_noise_reduced_{timestamp}.wav"
            raw_filename = f"fan_raw_{timestamp}.wav"
            
            # Save to files (in current working directory)
            sf.write(processed_filename, processed_audio, self.frequency_sample)
            sf.write(raw_filename, raw_audio, self.frequency_sample)
            
            rospy.loginfo(f"Processed audio saved to {processed_filename}")
            rospy.loginfo(f"Raw audio saved to {raw_filename}")
            rospy.loginfo("Recording complete. Shutting down node.")
            
            # Shutdown the node
            rospy.signal_shutdown("Recording complete")
            
        except Exception as e:
            rospy.logerr(f"Error saving audio: {e}")
    
def main():
    recorder = ChunkRecorder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt, shutting down")
    
if __name__ == '__main__':
    main()