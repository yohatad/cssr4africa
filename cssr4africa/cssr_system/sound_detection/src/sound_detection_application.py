#!/usr/bin/env python3
import math
import rospy
import std_msgs.msg
import logging
import numpy as np
import matplotlib.pyplot as plt
from sound_detection.msg import sound_detection
from sound_detection_implementation import NSnet2Enhancer  # Update the import path to match your setup
from silero_vad import load_silero_vad, get_speech_timestamps
from pathlib import Path
from threading import Lock
from scipy.signal import resample

class soundDetectionNode:
    def __init__(self):
        try:
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

            self.speed_of_sound = 343.0         # Speed of sound in m/s
            self.distance_between_ears = 0.07   # Distance between microphones in meters

            # Initialize NSnet2Enhancer
            self.enhancer = NSnet2Enhancer(fs=self.fs)
            self.vad_model = load_silero_vad()  # Load Silero VAD model

            self.audio_sub  = rospy.Subscriber('/naoqi_driver/audio', sound_detection, self.audio_callback)
            self.signal_pub = rospy.Publisher('/soundDetection/signal', std_msgs.msg.Float32MultiArray, queue_size=10)
            self.local_pub  = rospy.Publisher('/soundDetection/direction', std_msgs.msg.Float32, queue_size=10)

            rospy.loginfo('Sound detection node initialized')

            logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
            self.logger = logging.getLogger('soundDetectionNode')
            
        except Exception as e:
            rospy.logerr(f'Failed to initialize sound detection node: {e}')
            raise

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
    
    def localize(self, sigIn_frontLeft, sigIn_frontRight):
        """
        Localizes the sound source based on the time delay between signals received by two microphones.

        Parameters:
        - sigIn_frontLeft: Signal from the front left microphone
        - sigIn_frontRight: Signal from the front right microphone
        """
        
        # Step 1: Normalize the signals
        normalized_frontLeft = self.normalize_signal(sigIn_frontLeft)
        normalized_frontRight = self.normalize_signal(sigIn_frontRight)

        # Step 2: Localize and estimate the direction of the sound source
        itd_front = self.gcc_phat(normalized_frontLeft, normalized_frontRight, fs=self.fs)

        # Step 3: Calculate the angle of arrival based on the ITD
        angle = self.calculate_angle(itd_front)

        print(f'Estimated angle of arrival: {angle:.2f} degrees')

        # Step 4: Publish the estimated angle of arrival to the 'soundDetection/direction' topic
        out_msg = std_msgs.msg.Float32()
        out_msg.data = angle
        self.local_pub.publish(out_msg)

    def  audio_callback(self, msg):
        # Step 1: Convert incoming message data to float32 format and normalize it to the range [-1, 1]
        sigIn_frontLeft = np.array(msg.frontLeft, dtype=np.float32) / 32767.0
        sigIn_frontRight = np.array(msg.frontRight, dtype=np.float32) / 32767.0

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

            # Step 5: Insert the new data at the end of the buffers
            self.frontleft_buffer[-new_data_length:] = sigIn_frontLeft
            self.frontright_buffer[-new_data_length:] = sigIn_frontRight

            # Step 6: If enough samples are accumulated (equal or more than the localization buffer size), process localization
            if self.accumulated_samples >= self.localization_buffer_size:
                # Check if a voice is detected in the buffer before performing sound localization
                if self.is_voice_detected(self.frontleft_buffer):
                    # Localize the sound source based on the signals in the buffers
                    self.localize(self.frontleft_buffer, self.frontright_buffer)
                
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
        """
        Use Silero VAD to detect voice activity in the audio frame.
        """
        # Step 1: Downsample the audio frame from 48000 Hz to 16000 Hz
        target_sample_rate = 16000
        num_samples = int(len(audio_frame) * (target_sample_rate / self.fs))
        downsampled_audio = resample(audio_frame, num_samples)

        # Step 2: Convert the downsampled audio to a format compatible with Silero VAD (int16 PCM)
        audio_int16 = (downsampled_audio * 32767).astype(np.int16)

        # Step 3: Run Silero VAD on the downsampled audio
        speech_timestamps = get_speech_timestamps(audio_int16, self.vad_model, sampling_rate=target_sample_rate)
        print(f'Speech timestamps: {speech_timestamps}')

        return len(speech_timestamps) > 0  # True if any speech segment is detected
    
    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rospy.init_node('soundDetection', anonymous=True)

        node = soundDetectionNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        logging.getLogger('soundDetectionNode').error(f'Unexpected error: {e}')
