"""
sound_detection_implementation.py Implementation code for running the sound detection and localization algorithm

Author: Yohannes Tadesse Haile
Date: April 10, 2025
Version: v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import math
import os
import json
import rospy
import std_msgs.msg
import webrtcvad
import rospkg
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from datetime import datetime
from cssr_system.msg import microphone_msg_file
from threading import Lock
from std_msgs.msg import Float32MultiArray
from scipy import signal
from threading import RLock

class SoundDetectionNode:
    """
    SoundDetectionNode processes audio data from a microphone topic, applies VAD to determine if speech is present,
    applies bandpass filtering and spectral subtraction on the left channel, and localizes the sound source by computing
    the interaural time difference (ITD) via GCC-PHAT.
    
    Modified to publish speech segments based on natural pauses and handle long utterances,
    with extended buffering before and after speech is detected.
    """
    def __init__(self):
        """
        Initialize the SoundDetectionNode.
        Sets up ROS subscribers, publishers, and loads configuration parameters.
        """
        # Set node name for consistent logging
        self.node_name = "sound_detection"
        
        # Get configuration parameters from the ROS parameter server
        self.config = rospy.get_param('/soundDetection_config', {})
        
        # Set parameters from config
        self.frequency_sample = 48000
        self.speed_of_sound = 343.0
        self.distance_between_ears = self.config.get('distanceBetweenEars', 0.07)
        self.intensity_threshold = self.config.get('intensityThreshold', 3.9e-3)
        self.verbose_mode = self.config.get('verboseMode', False)
        
        # Utterance detection parameters
        self.speech_timeout = self.config.get('speechTimeout', 1.0)                 # Time (seconds) of silence to end an utterance
        self.min_utterance_length = self.config.get('minUtteranceLength', 0.05)     # Minimum utterance duration in seconds
        
        # Parameters for sentence separation
        self.sentence_pause_threshold = self.config.get('sentencePauseThreshold', 0.5)  # Time (seconds) of silence to separate sentences
        self.max_continuous_speech = self.config.get('maxContinuousSpeech', 10.0)       # Max time (seconds) before forced separation
        self.max_continuous_samples = int(self.max_continuous_speech * self.frequency_sample)
        self.continuous_speech_count = 0  # Counter for continuous speech
        
        # Pre-buffer to store audio before speech is detected
        self.pre_buffer_duration = self.config.get('preBufferDuration', 0.3)  # Duration to keep before speech in seconds
        self.pre_buffer_size = int(self.pre_buffer_duration * self.frequency_sample)
        self.pre_speech_buffer = []  # Buffer to store audio before speech is detected

        # Post-buffer to include audio after speech ends
        self.post_buffer_duration = self.config.get('postBufferDuration', 0.2)  # Duration to keep after speech in seconds
        self.post_buffer_size = int(self.post_buffer_duration * self.frequency_sample)
        self.post_speech_collection = False  # Flag to indicate post-speech collection
        self.post_speech_samples = 0  # Counter for post-speech samples
        
        # Utterance state variables
        self.in_utterance = False
        self.last_speech_time = None
        self.utterance_audio_buffer = []  # Use a regular list instead of deque with maxlen
        self.utterance_lock = RLock()
        
        # Sample calculations
        self.min_utterance_samples = int(self.min_utterance_length * self.frequency_sample)
        
        self.enable_plot = rospy.get_param('/soundDetection_config/generatePlot', False)
        
        # Buffer for localization (2 channels)
        self.localization_buffer_size = self.config.get('localizationBufferSize', 8192)
        self.frontleft_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
        self.frontright_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
        self.accumulated_samples = 0

        if self.enable_plot:
            # Set up plotting parameters
            self.plot_interval = rospy.get_param('/soundDetection_config/plotInterval', 10.0)  # seconds
            self.last_plot_time = rospy.get_time()
            self.plot_buffer_size = int(self.frequency_sample * self.plot_interval)
            self.raw_plot_buffer = np.zeros(self.plot_buffer_size, dtype=np.float32)
            self.processed_plot_buffer = np.zeros(self.plot_buffer_size, dtype=np.float32)
            self.raw_buffer_index = 0
            self.processed_buffer_index = 0
            self.plot_lock = Lock()
            
            # Get the save directory
            rospack = rospkg.RosPack()
            try:
                pkg_path = rospack.get_path('unit_tests')
                self.plot_save_dir = os.path.join(pkg_path, 'sound_detection_test/data')
                # Create directory if it doesn't exist
                if not os.path.exists(self.plot_save_dir):
                    os.makedirs(self.plot_save_dir)
            except rospkg.ResourceNotFound:
                rospy.logerr(f"{self.node_name}: Package 'unit_tests' not found.")
                raise
            except Exception as e:
                rospy.logerr(f"{self.node_name}: Error creating plot save directory: {e}")
                raise
                
            if self.verbose_mode:
                rospy.loginfo(f"{self.node_name}: Sound detection plotting enabled, saving to {self.plot_save_dir}")

        # Initialize VAD with configurable aggressiveness mode
        self.vad_aggressiveness = self.config.get('vadAggressiveness', 3)
        self.vad = webrtcvad.Vad(self.vad_aggressiveness)
        self.vad_frame_duration = 0.02  # 20 ms (WebRTC VAD requires specific frame durations)
        self.vad_frame_size = int(self.frequency_sample * self.vad_frame_duration)

        # Retrieve the microphone topic from the configuration file
        microphone_topic = self.extract_topics('Microphone')
        if not microphone_topic:
            rospy.logerr(f"{self.node_name}: Microphone topic not found in topic file.")
            raise ValueError("Missing microphone topic configuration.")

        # Initialize thread lock for shared resources
        self.lock = Lock()
        
        # Timer for periodic status message
        self.last_status_time = rospy.get_time()

        # Set up ROS subscribers and publishers
        self.audio_sub = rospy.Subscriber(microphone_topic, microphone_msg_file, self.audio_callback)
        self.signal_pub = rospy.Publisher('/soundDetection/signal', std_msgs.msg.Float32MultiArray, queue_size=10)
        self.direction_pub = rospy.Publisher('/soundDetection/direction', std_msgs.msg.Float32, queue_size=10)
        
        if self.verbose_mode:
            rospy.loginfo(f"{self.node_name}: Sound detection node initialized with extended buffering.")
            rospy.loginfo(f"{self.node_name}: Utterance timeout: {self.speech_timeout}s, Sentence pause: {self.sentence_pause_threshold}s")
            rospy.loginfo(f"{self.node_name}: Pre-buffer: {self.pre_buffer_duration}s, Post-buffer: {self.post_buffer_duration}s")
            rospy.loginfo(f"{self.node_name}: Max continuous speech: {self.max_continuous_speech}s")

    @staticmethod
    def read_json_file(package_name):
        """
        Read and parse a JSON configuration file from the specified ROS package.
        
        Args:
            package_name (str): Name of the ROS package containing the config file
            
        Returns:
            dict: Configuration data from JSON file, or empty dict if file not found
        """
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path(package_name)
            
            # Determine the directory and file name based on the package name
            if package_name == 'unit_tests':
                directory = 'sound_detection_test/config'
                config_file = 'sound_detection_test_configuration.json'
            else:
                directory = 'sound_detection/config'
                config_file = 'sound_detection_configuration.json'
            
            config_path = os.path.join(package_path, directory, config_file)
            
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    data = json.load(file)
                    return data
            else:
                rospy.logerr(f"read_json_file: Configuration file not found at {config_path}")
                return {}
                
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"ROS package '{package_name}' not found: {e}")
            return {}
        except json.JSONDecodeError as e:
            rospy.logerr(f"Error parsing JSON configuration file: {e}")
            return {}
        except Exception as e:
            rospy.logerr(f"Unexpected error reading configuration file: {e}")
            return {}

    @staticmethod
    def extract_topics(topic_key):
        """
        Extract the topic name for a given key from the topics data file.
        
        Args:
            topic_key (str): Key to search for in the topics file
            
        Returns:
            str or None: The topic name if found, None otherwise
        """
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('cssr_system')
            config_path = os.path.join(package_path, 'sound_detection/data', 'pepper_topics.dat')
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
            rospy.logerr(f"ROS package 'cssr_system' not found: {e}")
        return None
    
    def plot_audio_signals(self, raw_data, processed_data):
        """
        Create a plot comparing raw and processed audio signals.
        
        Args:
            raw_data (np.ndarray): Raw input audio data
            processed_data (np.ndarray): Processed audio data
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Create the figure
        plt.figure(figsize=(12, 6))
        
        # Use the same time axis for both signals
        time_axis = np.linspace(0, self.plot_interval, self.plot_buffer_size, endpoint=False)
        
        # Normalize signals for better comparison
        if np.max(np.abs(raw_data)) > 0:
            raw_norm = raw_data / np.max(np.abs(raw_data))
        else:
            raw_norm = raw_data
            
        if np.max(np.abs(processed_data)) > 0:
            processed_norm = processed_data / np.max(np.abs(processed_data))
        else:
            processed_norm = processed_data
        
        # Plot both signals
        plt.plot(time_axis, raw_norm, label='Raw Signal', color='blue', alpha=0.7)
        plt.plot(time_axis, processed_norm, label='Processed Signal', color='orange', alpha=0.7)
        
        # Add title and labels
        plt.title('Audio Signal Comparison (Normalized)')
        plt.xlabel('Time (s)')
        plt.ylabel('Amplitude (Normalized)')
        plt.grid(True)
        plt.legend()
        
        # Add info text
        info_text = f"Raw buffer fill: {self.raw_buffer_index/self.plot_buffer_size*100:.1f}%, " \
                    f"Processed buffer fill: {self.processed_buffer_index/self.plot_buffer_size*100:.1f}%"
        plt.figtext(0.5, 0.01, info_text, ha='center', fontsize=9)
        
        # Save the plot
        plot_path = os.path.join(self.plot_save_dir, f'sound_detection_test_output_sound_comparison_{timestamp}.png')
        plt.savefig(plot_path, dpi=150)
        plt.close()
        
        if self.verbose_mode:
            rospy.loginfo(f"{self.node_name}: Audio comparison plot saved to: {plot_path}")

    def spectral_subtraction(self, noisy_signal, fs, noise_frames=None, n_fft=None, hop_length=None):
        # Get values from config or use defaults
        noise_frames = noise_frames or self.config.get('noiseFrames', 15)
        n_fft = n_fft or self.config.get('fftSize', 1024)
        hop_length = hop_length or self.config.get('hopLength', 512)
        alpha = self.config.get('noiseReductionAlpha', 1.0)  # Less than 1.0 = conservative
        floor_coeff = self.config.get('spectralFloorCoeff', 0.005)  # e.g., 1% of noise

        # STFT
        f, t, Zxx = signal.stft(noisy_signal, fs=fs, nperseg=n_fft, noverlap=n_fft - hop_length)
        magnitude = np.abs(Zxx)
        phase = np.angle(Zxx)

        # Noise estimate from early frames
        if len(magnitude[0]) > noise_frames:
            noise_estimate = np.mean(magnitude[:, :noise_frames], axis=1, keepdims=True)
        else:
            # Handle case where there aren't enough frames
            noise_estimate = np.mean(magnitude, axis=1, keepdims=True)

        # Subtract and apply flooring
        magnitude_clean = magnitude - alpha * noise_estimate
        spectral_floor = floor_coeff * noise_estimate
        magnitude_clean = np.maximum(magnitude_clean, spectral_floor)

        # Reconstruct signal
        Zxx_clean = magnitude_clean * np.exp(1j * phase)
        _, recovered_signal = signal.istft(Zxx_clean, fs=fs, nperseg=n_fft, noverlap=n_fft - hop_length)

        return recovered_signal
    
    def normalize_rms(self, signal, target_rms=0.1):
        rms = np.sqrt(np.mean(signal ** 2))
        return signal * (target_rms / (rms + 1e-6)) if rms > 0 else signal

    def apply_bandpass_and_spectral_subtraction(self, data, fs):
        """
        Apply bandpass filtering followed by spectral subtraction.
        
        Args:
            data (np.ndarray): Input audio data
            fs (int): Sampling frequency
            
        Returns:
            np.ndarray: Processed signal with bandpass filtering and spectral subtraction
        """
        # Bandpass filter parameters
        lowcut = self.config.get('lowcutFrequency', 300.0)  # Hz
        highcut = self.config.get('highcutFrequency', 3400.0)  # Hz

        # Normalize frequencies by Nyquist frequency
        nyq = 0.5 * fs
        low = lowcut / nyq
        high = highcut / nyq
        
        # Design bandpass filter
        b, a = signal.butter(N=2, Wn=[low, high], btype='bandpass')
        
        # Apply bandpass filter
        filtered_signal = signal.lfilter(b, a, data)
        
        # Apply spectral subtraction to the filtered signal
        processed_signal = self.spectral_subtraction(filtered_signal, fs)

        # Normalize the processed signal to a target RMS level
        processed_signal = self.normalize_rms(processed_signal, target_rms=0.1)

        # Apply noise gate to the processed signal
        processed_signal = self.apply_noise_gate(processed_signal, threshold=0.005)
        
        return processed_signal
    
    def apply_noise_gate(self, signal, threshold=0.02):
        return np.where(np.abs(signal) < threshold, 0.0, signal)

    def voice_detected(self, audio_frame):
        """
        Use Voice Activity Detection (VAD) to determine if voice is present.
        
        Args:
            audio_frame (np.ndarray): Audio frame to analyze
            
        Returns:
            bool: True if voice is detected, False otherwise
        """
        try:
            # Process the audio in VAD frame-sized chunks
            for start in range(0, len(audio_frame) - self.vad_frame_size + 1, self.vad_frame_size):
                frame = audio_frame[start:start + self.vad_frame_size]
                
                # Convert to int16 bytes for WebRTC VAD
                frame_bytes = (frame * 32767).astype(np.int16).tobytes()
                
                # Check if this frame contains speech
                if self.vad.is_speech(frame_bytes, self.frequency_sample):
                    return True
            return False
        except Exception as e:
            rospy.logwarn(f"{self.node_name}: Error in VAD processing: {e}")
            return False

    def audio_callback(self, msg):
        """
        Process incoming audio data from the microphone.
        
        Args:
            msg (microphone_msg_file): The audio data message
        """
        try:
            # Print a status message every 10 seconds
            current_time = rospy.get_time()
            if current_time - self.last_status_time >= 10:
                rospy.loginfo(f"{self.node_name}: running.")
                self.last_status_time = current_time
                
            # Process audio data
            sigIn_frontLeft, sigIn_frontRight = self.process_audio_data(msg)

            # Always maintain a pre-speech buffer, regardless of speech state
            self.pre_speech_buffer.extend(sigIn_frontLeft)
            if len(self.pre_speech_buffer) > self.pre_buffer_size:
                # Keep only the most recent samples in the pre-buffer
                self.pre_speech_buffer = self.pre_speech_buffer[-self.pre_buffer_size:]
            
            # If we're in post-speech collection mode, add samples to the main buffer
            if self.post_speech_collection:
                with self.utterance_lock:
                    self.utterance_audio_buffer.extend(sigIn_frontLeft)
                    self.post_speech_samples += len(sigIn_frontLeft)
                    
                    # Check if we've collected enough post-speech samples
                    if self.post_speech_samples >= self.post_buffer_size:
                        if self.verbose_mode:
                            rospy.loginfo(f"{self.node_name}: Post-speech collection complete")
                        # Process the complete utterance with post-speech samples
                        self.process_complete_utterance()
                        # Reset post-speech collection
                        self.post_speech_collection = False
                        self.post_speech_samples = 0
                        # Reset utterance buffer
                        self.utterance_audio_buffer = []
                        self.in_utterance = False
                    
                # Update localization buffers
                with self.lock:
                    self.update_buffers(sigIn_frontLeft, sigIn_frontRight)
                    
                # Return early if we're just collecting post-speech samples
                return
            
            # Check intensity threshold
            if not self.is_intense_enough(sigIn_frontLeft):
                # Still maintain utterance state
                self.handle_utterance_state(False)
                return

            # Perform VAD check to determine if speech is present
            is_speech = self.voice_detected(sigIn_frontLeft)
            
            # If this is the start of speech, include pre-buffer content
            if is_speech and not self.in_utterance:
                with self.utterance_lock:
                    # Add pre-speech buffer content to the beginning of the utterance buffer
                    self.utterance_audio_buffer = list(self.pre_speech_buffer)
                    # Then add the current frame
                    self.utterance_audio_buffer.extend(sigIn_frontLeft)
            elif is_speech or self.in_utterance:
                # Add to utterance buffer if speech is detected or we're in an utterance
                with self.utterance_lock:
                    self.utterance_audio_buffer.extend(sigIn_frontLeft)
            
            # Keep track of speech state for utterance detection
            self.handle_utterance_state(is_speech)
            
            # Update plotting buffers if plotting is enabled
            if self.enable_plot:
                with self.plot_lock:
                    # Add raw signal to plot buffer
                    data_length = len(sigIn_frontLeft)
                    if self.raw_buffer_index + data_length <= self.plot_buffer_size:
                        self.raw_plot_buffer[self.raw_buffer_index:self.raw_buffer_index + data_length] = sigIn_frontLeft
                        self.raw_buffer_index += data_length
                    else:
                        # Buffer is full or wrapping around
                        remaining = self.plot_buffer_size - self.raw_buffer_index
                        if remaining > 0:
                            self.raw_plot_buffer[self.raw_buffer_index:] = sigIn_frontLeft[:remaining]
                        self.raw_buffer_index = min(data_length - remaining, self.plot_buffer_size)
                        if self.raw_buffer_index > 0:
                            self.raw_plot_buffer[:self.raw_buffer_index] = sigIn_frontLeft[remaining:remaining + self.raw_buffer_index]

            # Update localization buffers
            with self.lock:
                self.update_buffers(sigIn_frontLeft, sigIn_frontRight)

            # Localization processing
            if self.accumulated_samples >= self.localization_buffer_size:
                # Only perform localization if we're currently in an utterance
                if self.in_utterance:
                    self.localize(self.frontleft_buffer, self.frontright_buffer)
                
                # Reset buffers for next batch
                with self.lock:
                    self.frontleft_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
                    self.frontright_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
                    self.accumulated_samples = 0

                # We don't publish processed signal samples here anymore
                # Instead, we'll publish complete utterances when detected
                
                # Plot the audio if enabled
                if self.enable_plot:
                    current_time = rospy.get_time()
                    if current_time - self.last_plot_time >= self.plot_interval:
                        if self.raw_buffer_index >= self.plot_buffer_size * 0.5 and self.processed_buffer_index >= self.plot_buffer_size * 0.5:
                            self.plot_audio_signals(self.raw_plot_buffer, self.processed_plot_buffer)
                            # Reset buffer indices after plotting
                            self.raw_buffer_index = 0
                            self.processed_buffer_index = 0
                            self.last_plot_time = current_time

        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error in audio_callback: {e}")

    def handle_utterance_state(self, is_speech):
        """
        Manage the utterance state based on current speech detection.
        This handles the state machine for detecting complete utterances and sentences.
        
        Args:
            is_speech (bool): True if speech is currently detected
        """
        current_time = rospy.get_time()
        
        if is_speech:
            # Speech detected
            if not self.in_utterance:
                # Start of a new utterance
                self.in_utterance = True
                self.continuous_speech_count = 0
                if self.verbose_mode:
                    rospy.loginfo(f"{self.node_name}: Utterance started")
            else:
                # Already in utterance, check if we need to force-segment due to length
                self.continuous_speech_count += len(self.utterance_audio_buffer)
                if self.continuous_speech_count >= self.max_continuous_samples:
                    if self.verbose_mode:
                        rospy.loginfo(f"{self.node_name}: Force-segmenting long continuous speech ({self.continuous_speech_count/self.frequency_sample:.2f}s)")
                    # Process and publish the current segment
                    self.process_complete_utterance()
                    # Reset continuous speech counter but keep in_utterance state
                    self.continuous_speech_count = 0
                    # Clear buffer but maintain utterance state
                    self.utterance_audio_buffer = []
            
            # Update the last speech time regardless
            self.last_speech_time = current_time
            
        elif self.in_utterance:
            # We were in an utterance, check if we need to end or segment it
            speech_gap = current_time - self.last_speech_time if self.last_speech_time else 0
            
            if speech_gap > self.speech_timeout:
                # End of utterance detected, but start post-speech collection
                if not self.post_speech_collection:
                    if self.verbose_mode:
                        rospy.loginfo(f"{self.node_name}: Speech gap detected ({speech_gap:.2f}s), collecting post-speech samples")
                    # Start post-speech collection
                    self.post_speech_collection = True
                    self.post_speech_samples = 0
                
            elif speech_gap > self.sentence_pause_threshold:
                # Sentence pause detected, segment the current utterance but include post-buffer
                if not self.post_speech_collection:
                    if self.verbose_mode:
                        rospy.loginfo(f"{self.node_name}: Sentence pause detected ({speech_gap:.2f}s), collecting post-speech samples")
                    # Start post-speech collection
                    self.post_speech_collection = True
                    self.post_speech_samples = 0

    def process_complete_utterance(self):
        """
        Process and publish a complete utterance from the buffer.
        This is called when an utterance is detected as complete or when a sentence boundary is detected.
        """
        with self.utterance_lock:
            # Convert buffer to numpy array
            if len(self.utterance_audio_buffer) < self.min_utterance_samples:
                if self.verbose_mode:
                    rospy.loginfo(f"{self.node_name}: Utterance too short, discarding ({len(self.utterance_audio_buffer)/self.frequency_sample:.2f}s)")
                return
                
            utterance_data = np.array(self.utterance_audio_buffer, dtype=np.float32)
            
            # Apply processing to the entire utterance
            processed_utterance = self.apply_bandpass_and_spectral_subtraction(utterance_data, self.frequency_sample)

            # Publish the processed utterance to the original signal topic
            self.publish_signal(processed_utterance)
            
            if self.verbose_mode:
                duration = len(processed_utterance) / self.frequency_sample
                rospy.loginfo(f"{self.node_name}: Published speech segment ({duration:.2f}s) to /soundDetection/signal")

    def process_audio_data(self, msg):
        """
        Extract and normalize audio data from the message.
        
        Args:
            msg (microphone_msg_file): The audio data message
            
        Returns:
            tuple: (left_channel, right_channel) as normalized float32 arrays
        """
        try:
            # Convert int16 data to float32 and normalize to [-1.0, 1.0]
            sigIn_frontLeft = np.array(msg.frontLeft, dtype=np.float32) / 32767.0
            sigIn_frontRight = np.array(msg.frontRight, dtype=np.float32) / 32767.0
            return sigIn_frontLeft, sigIn_frontRight
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error processing audio data: {e}")
            return (np.zeros(self.localization_buffer_size, dtype=np.float32),
                    np.zeros(self.localization_buffer_size, dtype=np.float32))

    def is_intense_enough(self, signal_data):
        """
        Check if the signal intensity exceeds the threshold.
        
        Args:
            signal_data (np.ndarray): The audio signal data
            
        Returns:
            bool: True if signal is intense enough, False otherwise
        """
        # Calculate root mean square (RMS) intensity
        intensity = np.sqrt(np.mean(signal_data ** 2))
        return intensity >= self.intensity_threshold

    def update_buffers(self, sigIn_frontLeft, sigIn_frontRight):
        """
        Update the internal buffers with new audio data.
        
        Args:
            sigIn_frontLeft (np.ndarray): Left channel audio data
            sigIn_frontRight (np.ndarray): Right channel audio data
        """
        data_length = len(sigIn_frontLeft)
        if self.accumulated_samples + data_length <= self.localization_buffer_size:
            # There's room for all the new data
            start_index = self.accumulated_samples
            end_index = start_index + data_length
            self.frontleft_buffer[start_index:end_index] = sigIn_frontLeft
            self.frontright_buffer[start_index:end_index] = sigIn_frontRight
            self.accumulated_samples += data_length
        else:
            # Only part of the new data will fit
            remaining = self.localization_buffer_size - self.accumulated_samples
            if remaining > 0:
                self.frontleft_buffer[self.accumulated_samples:] = sigIn_frontLeft[:remaining]
                self.frontright_buffer[self.accumulated_samples:] = sigIn_frontRight[:remaining]
                self.accumulated_samples = self.localization_buffer_size

    def localize(self, sigIn_frontLeft, sigIn_frontRight):
        """
        Localize the sound source using the GCC-PHAT algorithm.
        
        Args:
            sigIn_frontLeft (np.ndarray): Left channel audio data
            sigIn_frontRight (np.ndarray): Right channel audio data
        """
        try:
            # Calculate Interaural Time Difference (ITD)
            itd = self.gcc_phat(sigIn_frontLeft, sigIn_frontRight, self.frequency_sample)
            
            # Convert ITD to angle
            angle = self.calculate_angle(itd)
            
            # Publish the calculated angle
            self.publish_angle(angle)
        except Exception as e:
            rospy.logwarn(f"{self.node_name}: Error in localization: {e}")

    def gcc_phat(self, sig, ref_sig, fs, max_tau=None, interp=16):
        """
        Implement the GCC-PHAT algorithm for time delay estimation.
        
        Args:
            sig (np.ndarray): Signal from first channel
            ref_sig (np.ndarray): Signal from reference channel
            fs (int): Sampling frequency
            max_tau (float, optional): Maximum delay to consider
            interp (int, optional): Interpolation factor
            
        Returns:
            float: Estimated time delay in seconds
        """
        try:
            # Compute FFT length
            n = sig.shape[0] + ref_sig.shape[0]
            
            # Compute FFTs
            SIG = np.fft.rfft(sig, n=n)
            REFSIG = np.fft.rfft(ref_sig, n=n)
            
            # Compute cross-correlation in frequency domain
            R = SIG * np.conj(REFSIG)
            
            # Apply phase transform (PHAT)
            R /= (np.abs(R) + 1e-10)
            
            # Compute inverse FFT to get time-domain cross-correlation
            cc = np.fft.irfft(R, n=n)
            
            # Find maximum correlation
            max_shift = int(n / 2)
            if max_tau:
                max_shift = min(int(fs * max_tau), max_shift)
            
            # Concatenate the end and beginning of cc to align the shifts properly
            cc = np.concatenate((cc[-max_shift:], cc[:max_shift + 1]))
            
            # Find the shift that gives maximum correlation
            shift = np.argmax(np.abs(cc)) - max_shift
            
            # Convert shift to time
            return shift / float(fs)
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Error in GCC-PHAT: {e}")
            return 0

    def calculate_angle(self, itd):
        """
        Calculate the sound source angle from the ITD.
        
        Args:
            itd (float): Interaural Time Difference in seconds
            
        Returns:
            float: Sound source angle in degrees
        """
        try:
            # Calculate sine of the angle
            z = itd * (self.speed_of_sound / self.distance_between_ears)
            
            # Clamp value to valid range for arcsin
            z = max(-1.0, min(1.0, z))
            
            # Calculate angle in degrees
            angle = math.asin(z) * (180.0 / math.pi)
            return angle
        except ValueError as e:
            rospy.logwarn(f"{self.node_name}: Invalid ITD for angle calculation: {e}")
            return 0.0

    def publish_angle(self, angle):
        """
        Publish the calculated angle to the direction topic.
        
        Args:
            angle (float): Sound source angle in degrees
        """
        angle_msg = std_msgs.msg.Float32()
        angle_msg.data = angle
        self.direction_pub.publish(angle_msg)

    def publish_signal(self, signal_data):
        """
        Publish the processed signal to the signal topic.
        
        Args:
            signal_data (np.ndarray): Processed audio signal
        """
        signal_msg = Float32MultiArray()
        signal_msg.data = signal_data.tolist()
        self.signal_pub.publish(signal_msg)

    def spin(self):
        """
        Main processing loop for the node.
        """
        rospy.spin()