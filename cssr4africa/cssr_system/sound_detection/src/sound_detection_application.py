#!/usr/bin/env python3
import math
import rospy
import std_msgs.msg
import webrtcvad
import rospkg
import logging
import os
import numpy as np
from sound_detection.msg import sound_detection
from sound_detection_implementation import NSnet2Enhancer
from threading import Lock
from std_msgs.msg import Float32MultiArray

class soundDetectionNode:
    def __init__(self):
        try:
            rospy.init_node('soundDetection', anonymous=True)
            self.fs = 48000  
            self.save_interval = 30
            self.audio_frame_duration = 0.25
            self.max_buffer_duration = 10
            self.intensity_threshold = 3.9e-3
            self.buffer_size = int(self.fs * self.max_buffer_duration)
            self.audio_buffer = np.zeros(self.buffer_size, dtype=np.float32)

            self.audio_buffer2 = []
            self.processing_audio_buffer2 = []
            
            self.buffer_start = 0
            self.buffer_end = 0
            self.processed_audio_buffer = np.array([], dtype=np.float32)
            self.lock = Lock()
            
            self.localization_buffer_size = 8192
            self.accumulated_samples = 0
            self.frontleft_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
            self.frontright_buffer = np.zeros(self.localization_buffer_size, dtype=np.float32)
            
            self.speed_of_sound = 343.0
            self.distance_between_ears = 0.07

            rospack = rospkg.RosPack()
            model_path = rospack.get_path('sound_detection') + '/models/sound_detection_nsnet2-20ms-48k.onnx'
            
            self.enhancer = NSnet2Enhancer(fs=self.fs, model_path=model_path)
            self.vad = webrtcvad.Vad(3)
            
            self.vad_frame_duration = 0.02
            self.vad_frame_size = int(self.fs * self.vad_frame_duration)
            
            self.audio_sub = rospy.Subscriber('/naoqi_driver/audio', sound_detection, self.audio_callback)
            self.signal_pub = rospy.Publisher('/soundDetection/signal', std_msgs.msg.Float32MultiArray, queue_size=10)
            self.local_pub = rospy.Publisher('/soundDetection/direction', std_msgs.msg.Float32, queue_size=10)
            
            rospy.loginfo('Sound detection node initialized')
            logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
            self.logger = logging.getLogger('soundDetectionNode')
        
        except Exception as e:
            rospy.logerr(f'Failed to initialize sound detection node: {e}')
            raise

    def resolve_model_path(self, path):
        if path.startswith('package://'):
            path = path[len('package://'):]
            package_name, relative_path = path.split('/', 1)
            rospack = rospkg.RosPack()
            package_path = rospack.get_path(package_name)
            path = os.path.join(package_path, relative_path)
        return path

    def audio_callback(self, msg):
        try:
            sigIn_frontLeft, sigIn_frontRight = self.process_audio_data(msg)
            if not self.is_intense_enough(sigIn_frontLeft):
                return
            with self.lock:
                self.update_buffers(sigIn_frontLeft, sigIn_frontRight)
            if self.accumulated_samples >= self.localization_buffer_size:
                if self.is_voice_detected(self.frontleft_buffer):
                    self.localize(self.frontleft_buffer, self.frontright_buffer)
                self.accumulated_samples = 0
            
            signal_fontleft = np.array(msg.frontLeft)

            with self.lock:
                self.audio_buffer2.extend(signal_fontleft.tolist())
                self.processing_audio_buffer2.extend(signal_fontleft.tolist())

            if len(self.audio_buffer2) >= self.fs:
                self.process_audio2()

        except Exception as e:
            rospy.logerr(f"Error in audio_callback: {e}")
            self.logger.error(f"Error in audio_callback: {e}")

    def process_audio2(self):
        with self.lock:
            sigIn = np.array(self.audio_buffer2[:self.fs ]) / 32767
            self.audio_buffer = self.audio_buffer2[self.fs:]

        outSig = self.enhancer(sigIn, self.fs)
        with self.lock:
            self.processing_audio_buffer2.extend(outSig.tolist())

        out_msg = Float32MultiArray(data=outSig)
        self.signal_pub.publish(out_msg)

    def process_audio_data(self, msg):
        """ Convert incoming message data to float32 format and normalize to [-1, 1]. """
        try:
            sigIn_frontLeft = np.array(msg.frontLeft, dtype=np.float32) / 32767.0
            sigIn_frontRight = np.array(msg.frontRight, dtype=np.float32) / 32767.0
            return sigIn_frontLeft, sigIn_frontRight
        except Exception as e:
            rospy.logerr(f"Error processing audio data: {e}")
            return np.zeros(self.localization_buffer_size), np.zeros(self.localization_buffer_size)

    def is_intense_enough(self, signal):
        """ Check if the signal intensity meets the threshold. """
        intensity = np.sqrt(np.mean(signal ** 2))
        return intensity >= self.intensity_threshold

    def update_buffers(self, sigIn_frontLeft, sigIn_frontRight):
        """ Update rolling buffers with new data and avoid overflow. """
        
        new_data_length = len(sigIn_frontLeft)
        
        if new_data_length > self.localization_buffer_size:
            rospy.logwarn("New data length exceeds buffer size, truncating.")
            sigIn_frontLeft = sigIn_frontLeft[-self.localization_buffer_size:]
            sigIn_frontRight = sigIn_frontRight[-self.localization_buffer_size:]
        
        self.frontleft_buffer = np.roll(self.frontleft_buffer, -new_data_length)
        self.frontright_buffer = np.roll(self.frontright_buffer, -new_data_length)
        self.frontleft_buffer[-new_data_length:] = sigIn_frontLeft
        self.frontright_buffer[-new_data_length:] = sigIn_frontRight
        self.accumulated_samples += new_data_length

    def check_audio_buffer_ready(self):
        """ Check if enough audio samples have accumulated in the general buffer. """
        return (self.buffer_end - self.buffer_start) % self.buffer_size >= int(self.fs * self.audio_frame_duration)

    def process_audio(self):
        try:
            frame_size = int(self.fs * self.audio_frame_duration)
            with self.lock:
                sigIn = self.extract_buffer_frame(frame_size)
                outSig = self.enhancer(sigIn, self.fs)
                self.processed_audio_buffer = np.concatenate((self.processed_audio_buffer, outSig))
                out_msg = std_msgs.msg.Float32MultiArray(data=outSig)
                self.signal_pub.publish(out_msg)
        except Exception as e:
            rospy.logerr(f"Error in process_audio: {e}")

    def extract_buffer_frame(self, frame_size):
        """ Extract a frame from the general audio buffer with buffer overflow checks. """
        if self.buffer_end >= self.buffer_start:
            sigIn = self.audio_buffer[self.buffer_start:self.buffer_start + frame_size]
        else:
            end_size = self.buffer_size - self.buffer_start
            sigIn = np.concatenate((self.audio_buffer[self.buffer_start:], self.audio_buffer[:frame_size - end_size]))
        self.buffer_start = (self.buffer_start + frame_size) % self.buffer_size
        return sigIn

    def is_voice_detected(self, audio_frame):
        """ Refined VAD frame processing with early detection. """
        try:
            for start in range(0, len(audio_frame) - self.vad_frame_size + 1, self.vad_frame_size):
                frame = audio_frame[start:start + self.vad_frame_size]
                frame_bytes = (frame * 32767).astype(np.int16).tobytes()
                if self.vad.is_speech(frame_bytes, self.fs):
                    return True
            return False
        except Exception as e:
            rospy.logwarn(f"Error in VAD processing: {e}")
            return False

    def localize(self, sigIn_frontLeft, sigIn_frontRight):
        try:
            normalized_frontLeft = self.normalize_signal(sigIn_frontLeft)
            normalized_frontRight = self.normalize_signal(sigIn_frontRight)
            itd = self.gcc_phat(normalized_frontLeft, normalized_frontRight, self.fs)
            angle = self.calculate_angle(itd)
            rospy.loginfo(f'Angle of arrival: {angle:.2f} degrees')
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
            angle = math.asin(z) * (180.0 / np.pi)
            return angle
        except ValueError as e:
            rospy.logwarn(f"Invalid ITD for angle calculation: {e}")
            return 0.0

    def publish_angle(self, angle):
        angle_msg = std_msgs.msg.Float32()
        angle_msg.data = angle
        self.local_pub.publish(angle_msg)

    def normalize_signal(self, signal):
        """ Normalize the signal to prevent overflows. """
        try:
            return signal / np.max(np.abs(signal))
        except ValueError as e:
            rospy.logwarn(f"Normalization error: {e}")
            return signal

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
