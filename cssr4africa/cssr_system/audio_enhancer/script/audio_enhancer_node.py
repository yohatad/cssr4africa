#!/usr/bin/env python3
import rospy
from audio_enhancer.msg import AudioCustomMsg
from std_msgs.msg import Float32MultiArray
import numpy as np
import soundfile as sf
from pathlib import Path
from enhance_onnx import NSnet2Enhancer
import threading
import time
import logging
import webrtcvad

class AudioEnhancerNode:
    def __init__(self):
        rospy.init_node('audio_enhancer_node', anonymous=True)
        
        self.fs = rospy.get_param('~fs', 48000)
        self.model = rospy.get_param('~model', 'nsnet2-20ms-48k-baseline.onnx')
        
        self.output_dir = rospy.get_param('~output_dir', '/home/lab/workspace/pepper_rob_ws')
        self.save_interval = rospy.get_param('~save_interval', 30)
        self.audio_frame_duration = rospy.get_param('~audio_frame_duration', 0.25)
        self.max_buffer_duration = rospy.get_param('~max_buffer_duration', 10)

        self.buffer_size = int(self.fs * self.max_buffer_duration)
        self.audio_buffer = np.zeros(self.buffer_size, dtype=np.float32)
        self.buffer_start = 0
        self.buffer_end = 0
        self.processed_audio_buffer = np.array([], dtype=np.float32)
        self.lock = threading.Lock()

        self.enhancer = NSnet2Enhancer(fs=self.fs)
        self.vad = webrtcvad.Vad(3)  # Set aggressiveness level (0-3)

        self.vad_frame_duration = 0.02  # 20ms frames for VAD
        self.vad_frame_size = int(self.fs * self.vad_frame_duration)

        self.audio_sub = rospy.Subscriber('/naoqi_driver/audio', AudioCustomMsg, self.audio_callback)
        self.audio_pub = rospy.Publisher('/audio_enhancer', Float32MultiArray, queue_size=10)

        Path(self.output_dir).mkdir(parents=True, exist_ok=True)
        rospy.loginfo('Audio Enhancer Node initialized')

        # self.timer = rospy.Timer(rospy.Duration(self.save_interval), self.save_audio)

        logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
        self.logger = logging.getLogger('AudioEnhancerNode')

    def audio_callback(self, msg):
        sigIn = np.array(msg.frontLeft, dtype=np.float32) / 32767.0

        with self.lock:
            for sample in sigIn:
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

        out_msg = Float32MultiArray(data=outSig)
        self.audio_pub.publish(out_msg)

    def is_voice_detected(self, audio_frame):
        if len(audio_frame) < self.vad_frame_size:
            return False
        
        for start in range(0, len(audio_frame) - self.vad_frame_size + 1, self.vad_frame_size):
            frame = audio_frame[start:start + self.vad_frame_size]
            frame_bytes = (frame * 32767).astype(np.int16).tobytes()
            if self.vad.is_speech(frame_bytes, self.fs):
                print('voice detected')
                return True
        return False

    def save_audio(self, event):
        with self.lock:
            if self.processed_audio_buffer.size > 0:
                outSig = self.processed_audio_buffer
                out_path = Path(self.output_dir) / f'enhanced_audio_{int(time.time())}.wav'
                try:
                    sf.write(str(out_path), outSig, self.fs)
                    rospy.loginfo(f'Processed audio saved to {out_path}')
                except Exception as e:
                    self.logger.error(f'Failed to save audio: {e}')
                self.processed_audio_buffer = np.array([], dtype=np.float32)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = AudioEnhancerNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        logging.getLogger('AudioEnhancerNode').error(f'Unexpected error: {e}')