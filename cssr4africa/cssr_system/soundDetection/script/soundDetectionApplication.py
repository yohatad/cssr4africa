#!/usr/bin/env python3
from soundDetection.msg import AudioCustomMsg
from soundDetectionImplementation import NSnet2Enhancer  # Update the import path to match your setup
from pathlib import Path
from threading import Lock
from scipy.signal import fftconvolve
import math
import time
import rospy
import std_msgs.msg
import webrtcvad
import logging
import numpy as np
import soundfile as sf
from scipy.signal import butter, lfilter, wiener



class soundDetectionNode:
    def __init__(self):
        try:
            rospy.init_node('soundDetection', anonymous=True)
            
            # Get ROS parameters and handle paths with pathlib
            self.fs = 48000  

            # Dynamically get the user's home directory
            default_output_dir = Path.home() / 'workspace/pepper_rob_ws'
            output_dir_path = rospy.get_param('~output_dir', str(default_output_dir))
            self.output_dir = Path(output_dir_path).resolve()  # Convert to absolute path

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

            self.speed_of_sound = 343.0         # Speed of sound in m/s
            self.distance_between_ears = 0.07   # Distance between microphones in meters
            self.intensity_threshold = 400      # Intensity threshold for detecting sound

            # Initialize NSnet2Enhancer
            self.enhancer = NSnet2Enhancer(fs=self.fs)
            self.vad = webrtcvad.Vad(3)         # Set aggressiveness level (0-3)

            self.vad_frame_duration = 0.02      # 20ms frames for VAD
            self.vad_frame_size = int(self.fs * self.vad_frame_duration)

            self.audio_sub  = rospy.Subscriber('/naoqi_driver/audio', AudioCustomMsg, self.audio_callback)
            self.signal_pub = rospy.Publisher('/soundDetection/signal', std_msgs.msg.Float32MultiArray, queue_size=10)
            self.local_pub  = rospy.Publisher('/soundDetection/direction', std_msgs.msg.Float32, queue_size=10)

            # self.output_dir.mkdir(parents=True, exist_ok=True)
            rospy.loginfo('Sound detection node initialized')

            # self.timer = rospy.Timer(rospy.Duration(self.save_interval), self.save_audio)
            logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
            self.logger = logging.getLogger('soundDetectionNode')


            # Define sub-band frequencies
            self.sub_bands = [
                (300, 1000),   # Sub-band 1
                (1000, 2000),  # Sub-band 2
                (2000, 3400)   # Sub-band 3
            ]

            self.lowcut = 300
            self.highcut = 3400
            self.window_size = 4096
            
        except Exception as e:
            rospy.logerr(f'Failed to initialize sound detection node: {e}')
            raise
  
    # Define pre-processing functions
    def butter_bandpass(self, lowcut, highcut, fs, order=5):
        nyquist = 0.5 * fs
        low = lowcut / nyquist
        high = highcut / nyquist
        b, a = butter(order, [low, high], btype='band')
        return b, a

    def bandpass_filter(self, data, lowcut, highcut, fs, order=5):
        b, a = self.butter_bandpass(lowcut, highcut, fs, order=order)
        return lfilter(b, a, data)

    def normalize_signal(self, signal):
        return signal / np.max(np.abs(signal))

    def voice_activity_detection(self, signal, threshold=0.01):
        return np.where(np.abs(signal) > threshold, signal, 0)
    
    def create_sub_band_filters(self, sub_bands, fs, order=4):
        filters = []
        for low, high in sub_bands:
            b, a = self.butter_bandpass(low, high, fs, order=order)
            filters.append((b, a))
        return filters

    def apply_filter_bank(self, signal, filters):
        sub_band_signals = []
        for b, a in filters:
            filtered = lfilter(b, a, signal)
            sub_band_signals.append(filtered)
        return sub_band_signals
     
    def gcc_phat(self, sig, ref_sig, fs, max_tau=None, interp=16):
        n = sig.shape[0] + ref_sig.shape[0]
        SIG = np.fft.rfft(sig, n=n)
        REFSIG = np.fft.rfft(ref_sig, n=n)
        R = SIG * np.conj(REFSIG)
        R /= np.abs(R)
        cc = np.fft.irfft(R, n=n)
        max_shift = int(n / 2)
        if max_tau:
            max_shift = min(int(fs * max_tau), max_shift)
        cc = np.concatenate((cc[-max_shift:], cc[:max_shift+1]))
        shift = np.argmax(np.abs(cc)) - max_shift
        tau = shift / float(fs)
        return tau
    
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
        
        # Step 1: Noise reduction
        noise_reduced_frontLeft = wiener(sigIn_frontLeft)
        noise_reduced_frontRight = wiener(sigIn_frontRight)

        # Step 2: Bandpass filtering
        filtered_frontLeft = self.bandpass_filter(noise_reduced_frontLeft, self.lowcut, self.highcut, self.fs)
        filtered_frontRight = self.bandpass_filter(noise_reduced_frontRight, self.lowcut, self.highcut, self.fs)

        # Step 3: Normalize the signals
        normalized_frontLeft = self.normalize_signal(filtered_frontLeft)
        normalized_frontRight = self.normalize_signal(filtered_frontRight)

        # Step 4: Winowing (Hanning window)
        win = np.hanning(self.window_size)
        windowed_frontLeft = win * normalized_frontLeft[:self.window_size] 
        windowed_frontRight = win * normalized_frontRight[:self.window_size] 

        # Step 5: Reverberation suppression
        reverb_suppressed_frontleft = lfilter([1], [1, -0.95], windowed_frontLeft)
        reverb_suppressed_frontright = lfilter([1], [1, -0.95], windowed_frontRight)

        # Step 6: Voice activity detection
        vad_frontleft = self.voice_activity_detection(reverb_suppressed_frontleft)
        vad_frontright = self.voice_activity_detection(reverb_suppressed_frontright)

        # Step 7: Sub-band filtering
        filters = self.create_sub_band_filters(self.sub_bands, self.fs)
        sub_band_signals_frontleft = self.apply_filter_bank(vad_frontleft, filters)
        sub_band_signals_frontright = self.apply_filter_bank(vad_frontright, filters)

        print('Sub-band signals:', len(sub_band_signals_frontleft))

        # ITD estimation for each sub-band
        itds_estimates = []
        for i, (left, right) in enumerate(zip(sub_band_signals_frontleft, sub_band_signals_frontright)):
            itd = self.gcc_phat(left, right, fs=self.fs)
            itds_estimates.append(itd)

        # Step 8: Combine ITD estimates
        energies = [np.sum(np.abs(left)**2) for left in sub_band_signals_frontleft]
        itd_weights = [energy / sum(energies) for energy in energies]
        itd_combined = sum([itd * weight for itd, weight in zip(itds_estimates, itd_weights)])

        # Step 9: Calculate the angle of arrival
        angle = self.calculate_angle(itd_combined)
        print(f'Angle of arrival: {angle:.2f} degrees')

        # Step 10: Publish the calculated angle
        angle_msg = std_msgs.msg.Float32()
        angle_msg.data = angle
        self.local_pub.publish(angle_msg)

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
                if self.is_voice_detected(self.frontleft_buffer):
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
        
        # check if voice is detected and print message
        if self.is_voice_detected(outSig):
            print('voice detected')

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
