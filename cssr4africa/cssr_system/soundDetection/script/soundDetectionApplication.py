#!/usr/bin/env python3
from soundDetection.msg import AudioCustomMsg
from soundDetectionImplementation import NSnet2Enhancer  # Update the import path to match your setup
from pathlib import Path
from threading import Lock
import math
import time
import rospy
import std_msgs.msg
import webrtcvad
import logging
import numpy as np
import matplotlib.pyplot as plt

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
            self.intensity_threshold = 3.9e-3

            self.buffer_size = int(self.fs * self.max_buffer_duration)
            self.audio_buffer = np.zeros(self.buffer_size, dtype=np.float32)
            self.buffer_start = 0
            self.buffer_end = 0
            self.processed_audio_buffer = np.array([], dtype=np.float32)
            self.lock = Lock()
            self.localization_buffer_size = 8192
            self.accumulated_samples = 0

            self.frontleft_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
            self.frontright_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
            self.rearleft_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
            self.rearright_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)

            self.speed_of_sound = 343.0         # Speed of sound in m/s
            self.distance_between_ears = 0.07   # Distance between microphones in meters

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
            
        except Exception as e:
            rospy.logerr(f'Failed to initialize sound detection node: {e}')
            raise

    def plot_signals(self, original_signal, filtered_signal, mic_label):
        """
        Plots and saves the original and filtered signals.

        Parameters:
        - original_signal: The original unfiltered signal
        - filtered_signal: The signal after bandpass filtering
        - mic_label: Label indicating which microphone the signal is from
        """

        time_axis = np.arange(len(original_signal)) / self.fs

        plt.figure(figsize=(12, 6))

        # Plot original signal
        plt.subplot(2, 1, 1)
        plt.plot(time_axis, original_signal, label='Original Signal')
        plt.title(f'Original Signal - {mic_label}')
        plt.xlabel('Time [s]')
        plt.ylabel('Amplitude')
        plt.legend()
        plt.grid(True)

        # Plot filtered signal
        plt.subplot(2, 1, 2)
        plt.plot(time_axis, filtered_signal, label='Filtered Signal', color='orange')
        plt.title(f'Filtered Signal - {mic_label}')
        plt.xlabel('Time [s]')
        plt.ylabel('Amplitude')
        plt.legend()
        plt.grid(True)

        # Adjust layout and save the figure
        plt.tight_layout()

        # Create a directory to save the plots if it doesn't exist
        plot_dir = self.output_dir / 'plots'
        plot_dir.mkdir(parents=True, exist_ok=True)

        # Generate a filename with timestamp
        timestamp = int(time.time() * 1000)
        filename = plot_dir / f'{mic_label}_signal_{timestamp}.png'

        plt.savefig(str(filename))
        plt.close()
        rospy.loginfo(f'Plot saved to {filename}')

    def normalize_signal(self, signal):
        return signal / np.max(np.abs(signal))

    def voice_activity_detection(self, signal, threshold=0.01):
        return np.where(np.abs(signal) > threshold, signal, 0)
     
    def gcc_phat(self, sig, ref_sig, fs, max_tau=None, interp=16):
        n = sig.shape[0] + ref_sig.shape[0]
        SIG = np.fft.rfft(sig, n=n)
        REFSIG = np.fft.rfft(ref_sig, n=n)
        R = SIG * np.conj(REFSIG)
        R /= (np.abs(R) + 1e-10)  # Adding a small epsilon to avoid division by zero
        cc = np.fft.irfft(R, n=n)
        max_shift = int(n / 2)
        if max_tau:
            max_shift = min(int(fs * max_tau), max_shift)
        cc = np.concatenate((cc[-max_shift:], cc[:max_shift+1]))
        shift = np.argmax(np.abs(cc)) - max_shift
        tau = shift / float(fs)
        return tau
    
    def direction(self, sigIn_frontLeft, sigIn_frontRight, sigIn_rearLeft, sigIn_rearRight):
        
        """
        Estimate the direction of the sound source based on dynamically selected microphone pairs
        depending on whether the sound is coming from the front, back, left, or right.

        Parameters:
        - sigIn_frontLeft: Signal from the front-left microphone
        - sigIn_frontRight: Signal from the front-right microphone
        - sigIn_rearLeft: Signal from the rear-left microphone
        - sigIn_rearRight: Signal from the rear-right microphone

        Returns:
        - azimuth_angle: The estimated azimuth angle in degrees.
        - direction: The estimated direction (front, back, left, right).
        """

        # Step 1: Compute ITDs based on the general direction
        # Front/Back: Use FL-FR and BL-BR
        itd_front = self.gcc_phat(sigIn_frontLeft, sigIn_frontRight, fs=self.fs)
        itd_back = self.gcc_phat(sigIn_rearLeft, sigIn_rearRight, fs=self.fs)
        
        # Left/Right: Use FL-BL and FR-BR
        itd_left = self.gcc_phat(sigIn_frontLeft, sigIn_rearLeft, fs=self.fs)
        itd_right = self.gcc_phat(sigIn_frontRight, sigIn_rearRight, fs=self.fs)
        
        # rospy.loginfo(f'ITD (Front-Left to Front-Right): {itd_front * 1000:.5f} ms')
        # rospy.loginfo(f'ITD (Rear-Left to Rear-Right): {itd_back * 1000:.5f} ms')
        # rospy.loginfo(f'ITD (Front-Left to Rear-Left): {itd_left * 1000:.5f} ms')
        # rospy.loginfo(f'ITD (Front-Right to Rear-Right): {itd_right * 1000:.5f} ms')

        # Step 2: Determine the general direction using the ITDs
        if abs(itd_front) > abs(itd_left) and abs(itd_front) > abs(itd_right):
            if itd_front > 0:
                direction = 'right'
                # Use FL-FR and BL-BR for final azimuth estimation
                angle_front = self.calculate_angle(itd_front)
                angle_back = self.calculate_angle(itd_back)
                final_angle = (angle_front + angle_back) / 2.0
            else:
                direction = 'left'
                # Use FL-BL and FR-BR for final azimuth estimation
                angle_left = self.calculate_angle(itd_left)
                angle_right = self.calculate_angle(itd_right)
                final_angle = (angle_left + angle_right) / 2.0
        else:
            if itd_left > 0:
                direction = 'front'
                # Use FL-FR and BL-BR for final azimuth estimation
                angle_front = self.calculate_angle(itd_front)
                angle_back = self.calculate_angle(itd_back)
                final_angle = (angle_front + angle_back) / 2.0
            else:
                direction = 'back'
                # Use FL-BL and FR-BR for final azimuth estimation
                angle_left = self.calculate_angle(itd_left)
                angle_right = self.calculate_angle(itd_right)
                final_angle = (angle_left + angle_right) / 2.0

        rospy.loginfo(f'Final estimated direction: {direction} at angle: {final_angle:.2f} degrees')

        # Return the final angle and direction
        return final_angle, direction

    def calculate_angle(self, itd):
        """
        Calculate the angle of arrival of the sound source based on ITD.

        Parameters:
        - itd: Interaural Time Difference

        Returns:
        - angle: Angle of arrival in degrees
        """

        z = itd * (self.speed_of_sound / self.distance_between_ears)
        try:
            angle = math.asin(z) * (180.0 / np.pi)
        
        except ValueError:
            
            angle = 0.0
        
        return angle
    
    def localize(self, sigIn_frontLeft, sigIn_frontRight, sigIn_rearLeft, sigIn_rearRight):
        """
        Localizes the sound source based on the time delay between signals received by two microphones.

        Parameters:
        - sigIn_frontLeft: Signal from the front left microphone
        - sigIn_frontRight: Signal from the front right microphone
        """
        
        # Step 1: Normalize the signals
        normalized_frontLeft = self.normalize_signal(sigIn_frontLeft)
        normalized_frontRight = self.normalize_signal(sigIn_frontRight)
        normalized_rearLeft = self.normalize_signal(sigIn_rearLeft)
        normalized_rearRight = self.normalize_signal(sigIn_rearRight)

        # Step 2: Localize and estimate the direction of the sound source
        self.direction(normalized_frontLeft, normalized_frontRight, normalized_rearLeft, normalized_rearRight)

    def audio_callback(self, msg):
        # Step 1: Convert incoming message data to float32 format and normalize it to the range [-1, 1]
        sigIn_frontLeft = np.array(msg.frontLeft, dtype=np.float32) / 32767.0
        sigIn_frontRight = np.array(msg.frontRight, dtype=np.float32) / 32767.0
        sigIn_rearLeft = np.array(msg.rearLeft, dtype=np.float32) / 32767.0
        sigIn_rearRight = np.array(msg.rearRight, dtype=np.float32) / 32767.0

        # Step 2: Calculate the intensity of the front left signal (RMS) and skip processing if below threshold
        intensity_sigIn_frontLeft = np.sqrt(np.mean(sigIn_frontLeft ** 2))
        if intensity_sigIn_frontLeft < self.intensity_threshold:
            # Skip processing if the signal intensity is below the threshold
            return

        new_data_length = len(sigIn_frontLeft)  # Number of samples in this callback (expected to be 4096)

        with self.lock:
            # Step 3: Accumulate the number of samples we've processed since the last localization
            self.accumulated_samples += new_data_length

            # Step 4: Roll the front left and right buffers to make space for new data
            self.frontleft_buffer = np.roll(self.frontleft_buffer, -new_data_length)
            self.frontright_buffer = np.roll(self.frontright_buffer, -new_data_length)
            self.rearleft_buffer = np.roll(self.rearleft_buffer, -new_data_length)
            self.rearright_buffer = np.roll(self.rearright_buffer, -new_data_length)

            # Step 5: Insert the new data at the end of the buffers
            self.frontleft_buffer[-new_data_length:] = sigIn_frontLeft
            self.frontright_buffer[-new_data_length:] = sigIn_frontRight
            self.rearleft_buffer[-new_data_length:] = sigIn_rearLeft
            self.rearright_buffer[-new_data_length:] = sigIn_rearRight

            # Step 6: If enough samples are accumulated (equal or more than the localization buffer size), process localization
            if self.accumulated_samples >= self.localization_buffer_size:
                # Check if a voice is detected in the buffer before performing sound localization
                if self.is_voice_detected(self.frontleft_buffer):
                    # Localize the sound source based on the signals in the buffers
                    self.localize(self.frontleft_buffer, self.frontright_buffer, self.rearleft_buffer, self.rearright_buffer)
                
                # Reset the accumulated samples counter after processing the buffers
                self.accumulated_samples = 0

            # Step 7: Update the general audio buffer with the incoming front left signal for other audio processing
            for sample in sigIn_frontLeft:
                self.audio_buffer[self.buffer_end] = sample
                self.buffer_end = (self.buffer_end + 1) % self.buffer_size
                if self.buffer_end == self.buffer_start:
                    self.buffer_start = (self.buffer_start + 1) % self.buffer_size

        # Step 8: Trigger general audio processing when enough data is accumulated in the general buffer
        if (self.buffer_end - self.buffer_start) % self.buffer_size >= int(self.fs * self.audio_frame_duration):
            # Process accumulated audio data for other purposes (e.g., enhancement, saving, etc.)
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
