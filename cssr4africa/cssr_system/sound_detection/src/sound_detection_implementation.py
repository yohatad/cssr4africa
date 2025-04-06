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
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
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
        Sets up ROS subscribers, publishers, and loads configuration parameters.
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

        # Buffer for plotting every 10 seconds (10 sec * 48000 Hz)
        self.plot_buffer_size = self.frequency_sample * 10  
        self.plot_buffer = np.zeros(self.plot_buffer_size, dtype=np.float32)
        self.plot_buffer_index = 0

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
        f, t, Zxx = signal.stft(noisy_signal, fs=fs, nperseg=n_fft, noverlap=n_fft-hop_length)
        magnitude = np.abs(Zxx)
        phase = np.angle(Zxx)
        noise_estimate = np.mean(magnitude[:, :noise_frames], axis=1, keepdims=True)
        magnitude_clean = magnitude - noise_estimate
        magnitude_clean[magnitude_clean < 0] = 0
        Zxx_clean = magnitude_clean * np.exp(1j * phase)
        _, recovered_signal = signal.istft(Zxx_clean, fs=fs, nperseg=n_fft, noverlap=n_fft-hop_length)
        return recovered_signal

    def apply_bandpass_and_spectral_subtraction(self, data, fs):
        lowcut = 300.0
        highcut = 3400.0
        nyq = 0.5 * fs
        low = lowcut / nyq
        high = highcut / nyq
        b, a = signal.butter(N=6, Wn=[low, high], btype='bandpass')
        filtered_signal = signal.lfilter(b, a, data)
        processed_signal = self.spectral_subtraction(filtered_signal, fs)
        return processed_signal

    def audio_callback(self, msg):
        try:
            sigIn_frontLeft, sigIn_frontRight = self.process_audio_data(msg)

            if not self.is_intense_enough(sigIn_frontLeft):
                return

            with self.lock:
                # Update localization buffers
                self.update_buffers(sigIn_frontLeft, sigIn_frontRight)
                # Also update plot buffer with unfiltered data from front left
                data_length = len(sigIn_frontLeft)  # always 4096
                if self.plot_buffer_index + data_length <= self.plot_buffer_size:
                    self.plot_buffer[self.plot_buffer_index:self.plot_buffer_index + data_length] = sigIn_frontLeft
                    self.plot_buffer_index += data_length
                else:
                    remaining = self.plot_buffer_size - self.plot_buffer_index
                    if remaining > 0:
                        self.plot_buffer[self.plot_buffer_index:] = sigIn_frontLeft[:remaining]
                        self.plot_buffer_index = self.plot_buffer_size

            # When 10 seconds of data have been accumulated, process and plot
            if self.plot_buffer_index >= self.plot_buffer_size:
                # Apply filtering to the 10-second block
                filtered_plot_signal = self.apply_bandpass_and_spectral_subtraction(self.plot_buffer, self.frequency_sample)
                self.plot_signals(self.plot_buffer, filtered_plot_signal)
                # Reset the plot buffer for the next 10 seconds
                with self.lock:
                    self.plot_buffer_index = 0

            # Localization processing (using the 8192-sample buffers)
            if self.accumulated_samples >= self.localization_buffer_size:
                processed_signal = self.apply_bandpass_and_spectral_subtraction(self.frontleft_buffer, self.frequency_sample)
                self.publish_signal(processed_signal)
                if self.voice_detected(self.frontleft_buffer):
                    self.localize(self.frontleft_buffer, self.frontright_buffer)
                self.frontleft_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
                self.frontright_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
                self.accumulated_samples = 0

        except Exception as e:
            rospy.logerr(f"Error in audio_callback: {e}")

    def process_audio_data(self, msg):
        try:
            sigIn_frontLeft = np.array(msg.frontLeft, dtype=np.float32) / 32767.0
            sigIn_frontRight = np.array(msg.frontRight, dtype=np.float32) / 32767.0
            return sigIn_frontLeft, sigIn_frontRight
        except Exception as e:
            rospy.logerr(f"Error processing audio data: {e}")
            return (np.zeros(self.localization_buffer_size, dtype=np.float32),
                    np.zeros(self.localization_buffer_size, dtype=np.float32))

    def is_intense_enough(self, signal_data):
        intensity = np.sqrt(np.mean(signal_data ** 2))
        return intensity >= self.intensity_threshold

    def update_buffers(self, sigIn_frontLeft, sigIn_frontRight):
        data_length = len(sigIn_frontLeft)  # always 4096
        if self.accumulated_samples + data_length <= self.localization_buffer_size:
            start_index = self.accumulated_samples
            end_index = start_index + data_length
            self.frontleft_buffer[start_index:end_index] = sigIn_frontLeft
            self.frontright_buffer[start_index:end_index] = sigIn_frontRight
            self.accumulated_samples += data_length
        else:
            remaining = self.localization_buffer_size - self.accumulated_samples
            if remaining > 0:
                self.frontleft_buffer[self.accumulated_samples:] = sigIn_frontLeft[:remaining]
                self.frontright_buffer[self.accumulated_samples:] = sigIn_frontRight[:remaining]
                self.accumulated_samples = self.localization_buffer_size

    def voice_detected(self, audio_frame):
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
        try:
            itd = self.gcc_phat(sigIn_frontLeft, sigIn_frontRight, self.frequency_sample)
            angle = self.calculate_angle(itd)
            self.publish_angle(angle)
        except Exception as e:
            rospy.logwarn(f"Error in localization: {e}")

    def gcc_phat(self, sig, ref_sig, fs, max_tau=None, interp=16):
        try:
            n = sig.shape[0] + ref_sig.shape[0]
            SIG = np.fft.rfft(sig, n=n)
            REFSIG = np.fft.rfft(ref_sig, n=n)
            R = SIG * np.conj(REFSIG)
            R /= (np.abs(R) + 1e-10)
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
        try:
            z = itd * (self.speed_of_sound / self.distance_between_ears)
            z = max(-1.0, min(1.0, z))
            angle = math.asin(z) * (180.0 / math.pi)
            return angle
        except ValueError as e:
            rospy.logwarn(f"Invalid ITD for angle calculation: {e}")
            return 0.0

    def publish_angle(self, angle):
        angle_msg = std_msgs.msg.Float32()
        angle_msg.data = angle
        self.direction_pub.publish(angle_msg)

    def publish_signal(self, signal_data):
        signal_msg = Float32MultiArray()
        signal_msg.data = signal_data.tolist()
        self.signal_pub.publish(signal_msg)

    def plot_signals(self, unfiltered, filtered):
        """
        Plot the unfiltered and filtered signals for a 10-second duration.
        """
        # Ensure the filtered signal has the same length as the unfiltered signal
        if filtered.shape[0] > self.plot_buffer_size:
            filtered = filtered[:self.plot_buffer_size]
        elif filtered.shape[0] < self.plot_buffer_size:
            pad_length = self.plot_buffer_size - filtered.shape[0]
            filtered = np.concatenate((filtered, np.zeros(pad_length, dtype=filtered.dtype)))
        
        t = np.linspace(0, 10, self.plot_buffer_size, endpoint=False)
        plt.figure(figsize=(12, 6))
        plt.plot(t, unfiltered, label='Unfiltered Signal')
        plt.plot(t, filtered, label='Filtered Signal', alpha=0.7)
        plt.xlabel('Time (s)')
        plt.ylabel('Amplitude')
        plt.title('10-Second Audio Signal')
        plt.legend()
        plt.tight_layout()
        plt.savefig('/home/yoha/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/sound_detection/data/audio_signal_plot.png')
        plt.close()
        rospy.loginfo("Plotted 10-second audio signal.")

    def spin(self):
        rospy.spin()
