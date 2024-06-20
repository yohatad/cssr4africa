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

class AudioEnhancerNode:
    def __init__(self):
        rospy.init_node('audio_enhancer_node', anonymous=True)
        
        self.fs = 48000
        self.model = rospy.get_param('~model', 'nsnet2-20ms-48k-baseline.onnx')
        self.output_dir = rospy.get_param('~output_dir', '/home/yoha/workspace/pepper_rob_ws')
        self.audio_buffer = []
        self.input_audio_buffer = []
        self.processed_audio_buffer = []
        self.lock = threading.Lock()
        self.save_interval = 30  # Save processed audio every 10 seconds
        self.audio_frame_duration = 1  # Accumulate audio for 1 second before processing

        self.enhancer = NSnet2Enhancer(fs=self.fs)

        self.audio_sub = rospy.Subscriber('/naoqi_driver/audio', AudioCustomMsg, self.audio_callback)
        self.audio_pub = rospy.Publisher('/audio_enhancer', Float32MultiArray, queue_size=10)

        Path(self.output_dir).mkdir(parents=True, exist_ok=True)
        rospy.loginfo('Audio Enhancer Node initialized')

        # Timer to save audio every save_interval seconds
        self.timer = rospy.Timer(rospy.Duration(self.save_interval), self.save_audio)      

    def audio_callback(self, msg):
        sigIn = np.array(msg.frontLeft)
        
        with self.lock:
            self.audio_buffer.extend(sigIn.tolist())
            self.input_audio_buffer.extend(sigIn.tolist())

        # Check if we have accumulated enough audio data for 1 second
        if len(self.audio_buffer) >= self.fs * self.audio_frame_duration:
            self.process_audio()

    def process_audio(self):
        with self.lock:
            sigIn = np.array(self.audio_buffer[:self.fs * self.audio_frame_duration]) / 32767
            self.audio_buffer = self.audio_buffer[self.fs * self.audio_frame_duration:]

        outSig = self.enhancer(sigIn, self.fs)
        with self.lock:
            self.processed_audio_buffer.extend(outSig.tolist())

        out_msg = Float32MultiArray(data=outSig)
        self.audio_pub.publish(out_msg)

    def save_audio(self, event):
        with self.lock:
            if self.input_audio_buffer:
                inputSig = np.array(self.input_audio_buffer, dtype=np.int16)
                input_path = Path(self.output_dir) / f'input_audio_{int(time.time())}.wav'
                sf.write(str(input_path), inputSig, self.fs)
                rospy.loginfo(f'Input audio saved to {input_path}')
                self.input_audio_buffer.clear()
            
            if self.processed_audio_buffer:
                outSig = np.array(self.processed_audio_buffer, dtype=np.float32)
                out_path = Path(self.output_dir) / f'enhanced_audio_{int(time.time())}.wav'
                sf.write(str(out_path), outSig, self.fs)
                rospy.loginfo(f'Processed audio saved to {out_path}')
                self.processed_audio_buffer.clear()

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    node = AudioEnhancerNode()
    node.spin()
