"""
sound_implementation.py Implementation code for running the sound detection and localization algorithm

Author: Yohannes Tadesse Haile
Date: March 15, 2025
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
from sound_detection.msg import sound_detection
from threading import Lock
from std_msgs.msg import Float32MultiArray
from scipy import signal

class SoundDetectionNode:
    """
    SoundDetectionNode processes audio data from a microphone topic, applies VAD to determine if speech is present,
    applies bandpass filtering and spectral subtraction on the left channel, and localizes the sound source by computing
    the interaural time difference (ITD) via GCC-PHAT.
    """
    def __init__(self):
        """
        Initialize the SoundDetectionNode.
        
        This sets up ROS subscribers, publishers, and loads configuration parameters.
        """
        # Sampling parameters and physical constants
        self.frequency_sample = 48000
        self.speed_of_sound = 343.0
        self.distance_between_ears = 0.07
        
        # Buffer for localization (2 channels)
        self.localization_buffer_size = 8192
        self.frontleft_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
        self.frontright_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
        self.accumulated_samples = 0
        
        # Load configuration from JSON file
        config = self.read_json_file()
        if not config or 'intensity_threshold' not in config:
            rospy.logerr("Configuration not found or missing 'intensity_threshold'.")
            raise ValueError("Missing configuration data.")
        self.intensity_threshold = config.get('intensity_threshold', 3.9e-3)

        # Initialize VAD with aggressiveness mode 3
        self.vad = webrtcvad.Vad(3)
        self.vad_frame_duration = 0.02  # 20 ms
        self.vad_frame_size = int(self.frequency_sample * self.vad_frame_duration)
        
        # Retrieve the microphone topic from the configuration file
        microphone_topic = self.extract_topics('Microphone')
        if not microphone_topic:
            rospy.logerr("Microphone topic not found in topic file.")
            raise ValueError("Missing microphone topic configuration.")
        
        # Initialize thread lock for shared resources
        self.lock = Lock()
        
        # Set up ROS subscribers and publishers
        self.audio_sub = rospy.Subscriber(microphone_topic, sound_detection, self.audio_callback)
        self.signal_pub = rospy.Publisher('/soundDetection/signal', std_msgs.msg.Float32MultiArray, queue_size=10)
        self.direction_pub = rospy.Publisher('/soundDetection/direction', std_msgs.msg.Float32, queue_size=10)
        
        rospy.loginfo("Sound detection node initialized.")

    @staticmethod
    def read_json_file():
        """
        Read and return configuration parameters from a JSON file.
        
        The file is expected to be in the package's 'config' directory.
        """
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('sound_detection')
            config_path = os.path.join(package_path, 'config', 'sound_detection_configuration.json')
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    data = json.load(file) 
                    return data
            else:
                rospy.logerr(f"Configuration file not found at {config_path}")
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"ROS package 'sound_detection' not found: {e}")
        return None

    @staticmethod
    def extract_topics(topic_key):
        """
        Extract the topic name for a given key from the topics data file.
        
        The file is expected to be in the package's 'data' directory.
        
        :param topic_key: The key (e.g., 'Microphone') to search for.
        :return: The topic name as a string if found, else None.
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

    def spectral_subtraction(self, noisy_signal, fs, noise_frames=5, n_fft=1024, hop_length=512):
        """
        Perform spectral subtraction on a noisy signal.
        
        The first few frames (default: 5) are assumed to contain only noise, and their average spectrum is
        subtracted from all subsequent frames.
        
        :param noisy_signal: Input noisy signal (numpy array).
        :param fs: Sampling frequency (Hz).
        :param noise_frames: Number of initial frames to estimate the noise spectrum.
        :param n_fft: Number of FFT points.
        :param hop_length: Hop length between frames.
        :return: Recovered signal after spectral subtraction.
        """
        # Compute STFT of the noisy signal.
        f, t, Zxx = signal.stft(noisy_signal, fs=fs, nperseg=n_fft, noverlap=n_fft-hop_length)
        magnitude = np.abs(Zxx)
        phase = np.angle(Zxx)
        
        # Estimate noise magnitude spectrum from the first few frames.
        noise_estimate = np.mean(magnitude[:, :noise_frames], axis=1, keepdims=True)
        
        # Subtract the noise estimate from each frame's magnitude.
        magnitude_clean = magnitude - noise_estimate
        magnitude_clean[magnitude_clean < 0] = 0  # Ensure no negative magnitudes.
        
        # Reconstruct the cleaned complex spectrum.
        Zxx_clean = magnitude_clean * np.exp(1j * phase)
        
        # Inverse STFT to recover time-domain signal.
        _, recovered_signal = signal.istft(Zxx_clean, fs=fs, nperseg=n_fft, noverlap=n_fft-hop_length)
        return recovered_signal

    def apply_bandpass_and_spectral_subtraction(self, data, fs):
        """
        Apply bandpass filter followed by spectral subtraction on the input signal.

        :param data: Input signal (numpy array).
        :param fs: Sampling frequency.
        :return: Processed signal.
        """
        # Bandpass filter parameters
        lowcut = 300.0    # Low cutoff frequency in Hz
        highcut = 3400.0  # High cutoff frequency in Hz

        # Design bandpass filter using a 6th order Butterworth filter
        nyq = 0.5 * fs
        low = lowcut / nyq
        high = highcut / nyq
        b, a = signal.butter(N=6, Wn=[low, high], btype='bandpass')

        # Apply bandpass filter
        filtered_signal = signal.lfilter(b, a, data)

        # Apply spectral subtraction
        processed_signal = self.spectral_subtraction(filtered_signal, fs)

        # Ensure the processed signal has the same length as input
        processed_signal = processed_signal[:len(data)]

        return processed_signal

    def audio_callback(self, msg):
        """
        Callback function for incoming audio messages.
        
        Processes the audio data, updates the rolling buffers, and triggers localization if enough
        samples have been accumulated.
        """
        try:
            sigIn_frontLeft, sigIn_frontRight = self.process_audio_data(msg)
            
            # Check if the audio signal intensity is above the threshold.
            if not self.is_intense_enough(sigIn_frontLeft):
                return
            
            # Update buffers in a thread-safe manner.
            with self.lock:
                self.update_buffers(sigIn_frontLeft, sigIn_frontRight)

            # Apply filtering and spectral subtraction only on left channel
            sigIn_frontLeft_filtered = self.apply_bandpass_and_spectral_subtraction(self.frontleft_buffer, self.frequency_sample)

            # Publish the processed left signal
            self.publish_signal(sigIn_frontLeft_filtered)
            
            # If enough samples have been accumulated, process for voice detection and localization.
            if self.accumulated_samples >= self.localization_buffer_size:
                if self.voice_detected(self.frontleft_buffer):
                    self.localize(self.frontleft_buffer, self.frontright_buffer)
                self.accumulated_samples = 0
            
        except Exception as e:
            rospy.logerr(f"Error in audio_callback: {e}")

    def process_audio_data(self, msg):
        """
        Convert incoming message data to float32, normalize, and process the left channel
        with bandpass filtering and spectral subtraction.

        :param msg: The ROS message of type sound_detection.
        :return: Tuple (sigIn_frontLeft, sigIn_frontRight) as numpy arrays.
        """
        try:
            sigIn_frontLeft = np.array(msg.frontLeft, dtype=np.float32) / 32767.0
            sigIn_frontRight = np.array(msg.frontRight, dtype=np.float32) / 32767.0

            return sigIn_frontLeft, sigIn_frontRight
        
        except Exception as e:
            rospy.logerr(f"Error processing audio data: {e}")
            return (np.zeros(self.localization_buffer_size, dtype=np.float32),
                    np.zeros(self.localization_buffer_size, dtype=np.float32))

    def is_intense_enough(self, signal_data):
        """
        Determine whether the given signal meets the intensity threshold.
        
        :param signal_data: A numpy array containing audio data.
        :return: True if the signal intensity is above the threshold, False otherwise.
        """
        intensity = np.sqrt(np.mean(signal_data ** 2))
        return intensity >= self.intensity_threshold

    def update_buffers(self, sigIn_frontLeft, sigIn_frontRight):
        """
        Update the rolling buffers with new audio data.
        
        Rolls the current buffers to discard old data and appends the new data at the end.
        
        :param sigIn_frontLeft: New audio data from the left microphone.
        :param sigIn_frontRight: New audio data from the right microphone.
        """
        data_length = len(sigIn_frontLeft)
        
        # Roll the buffers and append new data.
        self.frontleft_buffer = np.roll(self.frontleft_buffer, -data_length)
        self.frontright_buffer = np.roll(self.frontright_buffer, -data_length)
        self.frontleft_buffer[-data_length:] = sigIn_frontLeft
        self.frontright_buffer[-data_length:] = sigIn_frontRight
        self.accumulated_samples += data_length

    def voice_detected(self, audio_frame):
        """
        Check if voice is detected within an audio frame using VAD.
        
        Processes the audio in frames and returns True if any frame is classified as speech.
        
        :param audio_frame: A numpy array containing audio data.
        :return: True if voice is detected, False otherwise.
        """
        try:
            for start in range(0, len(audio_frame) - self.vad_frame_size + 1, self.vad_frame_size):
                frame = audio_frame[start:start + self.vad_frame_size]
                frame_bytes = (frame * 32767).astype(np.int16).tobytes()
                if self.vad.is_speech(frame_bytes, self.frequency_sample):
                    return True
            return False
        except Exception as e:
            rospy.logwarn(f"Error in VAD processing: {e}")
            return False

    def localize(self, sigIn_frontLeft, sigIn_frontRight):
        """
        Perform sound source localization using GCC-PHAT and compute the angle of arrival.
        
        :param sigIn_frontLeft: Audio data from the left microphone.
        :param sigIn_frontRight: Audio data from the right microphone.
        """
        try:
            itd = self.gcc_phat(sigIn_frontLeft, sigIn_frontRight, self.frequency_sample)
            angle = self.calculate_angle(itd)
            self.publish_angle(angle)
        except Exception as e:
            rospy.logwarn(f"Error in localization: {e}")

    def gcc_phat(self, sig, ref_sig, fs, max_tau=None, interp=16):
        """
        Compute the Generalized Cross Correlation with Phase Transform (GCC-PHAT) to estimate the ITD.
        
        :param sig: Signal from one microphone.
        :param ref_sig: Signal from the other microphone.
        :param fs: Sampling frequency.
        :param max_tau: Optional maximum time delay.
        :param interp: Interpolation factor.
        :return: Estimated time delay in seconds.
        """
        try:
            n = sig.shape[0] + ref_sig.shape[0]
            SIG = np.fft.rfft(sig, n=n)
            REFSIG = np.fft.rfft(ref_sig, n=n)
            R = SIG * np.conj(REFSIG)
            R /= (np.abs(R) + 1e-10)  # Avoid division by zero
            cc = np.fft.irfft(R, n=n)
            max_shift = int(n / 2)
            if max_tau:
                max_shift = min(int(fs * max_tau), max_shift)
            cc = np.concatenate((cc[-max_shift:], cc[:max_shift + 1]))
            shift = np.argmax(np.abs(cc)) - max_shift
            return shift / float(fs)
        except Exception as e:
            rospy.logerr(f"Error in GCC-PHAT: {e}")
            return 0

    def calculate_angle(self, itd):
        """
        Calculate the angle of arrival based on the interaural time difference (ITD).
        
        :param itd: Interaural time difference in seconds.
        :return: Angle of arrival in degrees.
        """
        try:
            z = itd * (self.speed_of_sound / self.distance_between_ears)
            # Clamp z to the valid range for asin.
            z = max(-1.0, min(1.0, z))
            angle = math.asin(z) * (180.0 / math.pi)
            return angle
        except ValueError as e:
            rospy.logwarn(f"Invalid ITD for angle calculation: {e}")
            return 0.0

    def publish_angle(self, angle):
        """
        Publish the calculated angle of arrival on a ROS topic.
        
        :param angle: Angle of arrival in degrees.
        """
        angle_msg = std_msgs.msg.Float32()
        angle_msg.data = angle
        self.direction_pub.publish(angle_msg)

    def publish_signal(self, signal_data):
        """
        Publish the processed audio signal on a ROS topic.
        
        :param signal_data: Processed audio signal (numpy array).
        """
        signal_msg = Float32MultiArray()
        signal_msg.data = signal_data.tolist()
        self.signal_pub.publish(signal_msg)

    def spin(self):
        """
        Keep the node running.
        """
        rospy.spin()