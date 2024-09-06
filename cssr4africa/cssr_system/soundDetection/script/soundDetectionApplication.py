#!/usr/bin/env python3
import rospy
from soundDetection.msg import AudioCustomMsg
import std_msgs.msg
import numpy as np
import soundfile as sf
from pathlib import Path
from threading import Lock
from scipy.signal import fftconvolve
import time
import logging
import webrtcvad
import math
from soundDetectionImplementation import NSnet2Enhancer  # Update the import path to match your setup

class soundDetectionNode:
    def __init__(self):
        try:
            rospy.init_node('soundDetection', anonymous=True)

            # Dynamically get the user's home directory
            default_output_dir = Path.home() / 'workspace/pepper_rob_ws'
            output_dir_path = rospy.get_param('~output_dir', str(default_output_dir))
            self.output_dir = Path(output_dir_path).resolve()  # Convert to absolute path

            self.fs = 48000  
            self.save_interval = 30
            self.audio_frame_duration = 0.25
            self.max_buffer_duration = 10

            self.buffer_size = int(self.fs * self.max_buffer_duration)
            self.audio_buffer = np.zeros(self.buffer_size, dtype=np.float32)
            self.buffer_start = 0
            self.buffer_end = 0
            self.processed_audio_buffer = np.array([], dtype=np.float32)
            self.lock = Lock()
            self.localization_buffer_size = 8192
            self.frontleft_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
            self.frontright_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)

            self.speed_of_sound = 343.0  # Speed of sound in m/s
            self.distance_between_ears = 0.07  # Distance between microphones in meters
            self.intensity_threshold = 6.0846175e-05  # Intensity threshold for detecting sound

            # Initialize NSnet2Enhancer
            self.enhancer = NSnet2Enhancer(fs=self.fs)
            self.vad = webrtcvad.Vad(3)  # Set aggressiveness level (0-3)

            self.vad_frame_duration = 0.02  # 20ms frames for VAD
            self.vad_frame_size = int(self.fs * self.vad_frame_duration)

            self.audio_sub  = rospy.Subscriber('/naoqi_driver/audio', AudioCustomMsg, self.audio_callback)
            self.signal_pub = rospy.Publisher('/soundDetection/signal', std_msgs.msg.Float32MultiArray, queue_size=10)
            self.local_pub  = rospy.Publisher('/soundDetection/direction', std_msgs.msg.Float32, queue_size=10)

            # self.output_dir.mkdir(parents=True, exist_ok=True)
            rospy.loginfo('Sound detection node initialized')

            # self.timer = rospy.Timer(rospy.Duration(self.save_interval), self.save_audio)

            logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
            self.logger = logging.getLogger('soundDetectionNode')

        except Exception as e:
            rospy.logerr(f'Failed to initialize sound detection node: {e}')
            raise

    def gcc_phat(self, sig, refsig, fs=48000, max_tau=None, interp=16):
        """
        Compute the Generalized Cross-Correlation - Phase Transform (GCC-PHAT)
        to estimate time delay between two signals.

        Parameters:
        - sig: Signal array
        - refsig: Reference signal array
        - fs: Sampling frequency
        - max_tau: Maximum time delay to consider
        - interp: Interpolation factor for the cross-correlation

        Returns:
        - tau: Estimated time delay
        - cc: Cross-correlation function
        """
        n = sig.size + refsig.size

        # FFT of the signals
        SIG = np.fft.rfft(sig, n=n)
        REFSIG = np.fft.rfft(refsig, n=n)

        # Cross-spectral density
        R = SIG * np.conj(REFSIG)

        # Apply PHAT weighting
        R /= np.abs(R)

        # Inverse FFT to get cross-correlation
        cc = np.fft.irfft(R, n=(interp * n))

        # Find the peak
        max_shift = int(interp * fs * max_tau)
        cc = np.concatenate((cc[-max_shift:], cc[:max_shift + 1]))
        shift = np.argmax(np.abs(cc)) - max_shift
        tau = shift / float(interp * fs)

        return tau, cc
    
    def calculate_angle(self, itd):
        """
        Calculate the angle of arrival of the sound source based on ITD.

        Parameters:
        - itd: Interaural Time Difference

        Returns:
        - angle: Angle of arrival in degrees
        """
        z = itd * (self.speed_of_sound / self.distance_between_ears)
        angle = math.asin(z) * (180.0 / np.pi)
        return angle

    def localize(self, sigIn_frontLeft, sigIn_frontRight):
        """
        Localizes the sound source based on the time delay between signals received by two microphones.

        Parameters:
        - sigIn_frontLeft: Signal from the front left microphone
        - sigIn_frontRight: Signal from the front right microphone
        """
        # Compute the intensity of the sound source
        intensity = np.mean(sigIn_frontLeft ** 2 + sigIn_frontRight ** 2)

        # Check if the intensity is above the threshold
        if intensity > self.intensity_threshold:   
            # Compute the ITD using GCC-PHAT
            max_tau = self.distance_between_ears / self.speed_of_sound
            itd, _ = self.gcc_phat(sigIn_frontLeft, sigIn_frontRight, fs=self.fs, max_tau=max_tau)
            
            # Calculate the angle of arrival
            angle = self.calculate_angle(itd)
            
            # Publish the calculated angle
            angle_msg = std_msgs.msg.Float32()
            angle_msg.data = angle
            self.local_pub.publish(angle_msg)
            
            # Log the detected direction
            rospy.loginfo(f"Sound detected at angle: {angle:.2f} degrees")

    def audio_callback(self, msg):
        sigIn_frontLeft = np.array(msg.frontLeft, dtype=np.float32) / 32767.0
        sigIn_frontRight = np.array(msg.frontRight, dtype=np.float32) / 32767.0

        with self.lock:
            self.frontleft_buffer = np.roll(self.frontleft_buffer, -len(sigIn_frontLeft))
            self.frontright_buffer = np.roll(self.frontright_buffer, -len(sigIn_frontRight))

            self.frontleft_buffer[-len(sigIn_frontLeft):] = sigIn_frontLeft
            self.frontright_buffer[-len(sigIn_frontRight):] = sigIn_frontRight

            # Check if the buffers have reached the maximum size for localization
            if len(self.frontleft_buffer) >= self.localization_buffer_size and len(self.frontright_buffer) >= self.localization_buffer_size:
                # if self.is_voice_detected(self.frontleft_buffer):
                    self.localize(self.frontleft_buffer, self.frontright_buffer)

            for sample in sigIn_frontLeft:
                self.audio_buffer[self.buffer_end] = sample
                self.buffer_end = (self.buffer_end + 1) % self.buffer_size
                if self.buffer_end == self.buffer_start:
                    self.buffer_start = (self.buffer_start + 1) % self.buffer_size

        if (self.buffer_end - self.buffer_start) % self.buffer_size >= int(self.fs * self.audio_frame_duration):
            self.process_audio()

    def process_audio(self):
        frame_size = int(self.fs * self.audio_frame_duration)

        with self.lock:
            if self.buffer_end >= self.buffer_start:
                sigIn = self.audio_buffer[self.buffer_start:self.buffer_start + frame_size]
            else:
                end_size = self.buffer_size - self.buffer_start
                sigIn = np.concatenate((self.audio_buffer[self.buffer_start:], self.audio_buffer[:frame_size - end_size]))
            self.buffer_start = (self.buffer_start + frame_size) % self.buffer_size
        
        outSig = self.enhancer(sigIn, self.fs)
        
        with self.lock:
            self.processed_audio_buffer = np.concatenate((self.processed_audio_buffer, outSig))
        
        out_msg = std_msgs.msg.Float32MultiArray()
        self.signal_pub.publish(out_msg)

    def is_voice_detected(self, audio_frame):
        if len(audio_frame) < self.vad_frame_size:
            return False
        
        for start in range(0, len(audio_frame) - self.vad_frame_size + 1, self.vad_frame_size):
            frame = audio_frame[start:start + self.vad_frame_size]
            frame_bytes = (frame * 32767).astype(np.int16).tobytes()
            if self.vad.is_speech(frame_bytes, self.fs):
                return True
        return False

    # def save_audio(self, event):
    #     with self.lock:
    #         if self.processed_audio_buffer.size > 0:
    #             outSig = self.processed_audio_buffer
    #             out_path = self.output_dir / f'enhanced_audio_{int(time.time())}.wav'
    #             try:
    #                 sf.write(str(out_path), outSig, self.fs)
    #                 rospy.loginfo(f'Processed audio saved to {out_path}')
    #             except Exception as e:
    #                 self.logger.error(f'Failed to save audio: {e}')
    #             self.processed_audio_buffer = np.array([], dtype=np.float32)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = soundDetectionNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        logging.getLogger('soundDetectionNode').error(f'Unexpected error: {e}')
