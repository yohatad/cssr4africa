#!/usr/bin/env python

import qi
import argparse
import sys
import time
import numpy as np
import rospy
from naoqi_driver.msg import AudioCustomMsg

class SoundProcessingModule(object):
    def __init__(self, app, topic_name="/naoqi_driver/audio"):
        super(SoundProcessingModule, self).__init__()
        app.start()
        session = app.session
        self.audio_service = session.service("ALAudioDevice")  # Initialize audio service
        self.module_name = "SoundProcessingModule"
        self.micLeft = []
        self.micRight = []
        self.micFront = []
        self.micRear = []
        self.audioBuffer = []
        self.sampleRate = 48000

        # ROS setup with AudioData message type
        rospy.init_node('naoqi_audio_publisher', anonymous=True)
        self.pub = rospy.Publisher(topic_name, AudioCustomMsg, queue_size=10)

    def startProcessing(self):
        self.audio_service.setClientPreferences(self.module_name, self.sampleRate, 0, 1)
        self.audio_service.subscribe(self.module_name)

        while not rospy.is_shutdown():
            time.sleep(1)

        self.audio_service.unsubscribe(self.module_name)

    def processRemote(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        # Convert the inputBuffer to a numpy array and split it into 4 channels
        self.audioBuffer = np.frombuffer(inputBuffer, dtype=np.int16)
        self.micLeft = self.audioBuffer[nbOfSamplesByChannel*0:nbOfSamplesByChannel*1]
        self.micRight = self.audioBuffer[nbOfSamplesByChannel*1:nbOfSamplesByChannel*2]
        self.micFront = self.audioBuffer[nbOfSamplesByChannel*2:nbOfSamplesByChannel*3]
        self.micRear = self.audioBuffer[nbOfSamplesByChannel*3:nbOfSamplesByChannel*4]

        # Convert the numpy array to a byte array and publish it
        audio_data = AudioCustomMsg()
        audio_data.header.stamp = rospy.Time.now()
        audio_data.micLeft = self.micLeft.tobytes()
        audio_data.micRight = self.micRight.tobytes()
        audio_data.micFront = self.micFront.tobytes()
        audio_data.micRear = self.micRear.tobytes()

        self.pub.publish(audio_data)

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('naoqi_audio_publisher', anonymous=True)

    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="172.29.111.246", help="Robot IP address.")
    parser.add_argument("--port", type=int, default=9559, help="Naoqi port number")
    args, _ = parser.parse_known_args()

    try:
        connection_url = "tcp://" + args.ip + ":" + str(args.port)
        app = qi.Application(["SoundProcessingModule", "--qi-url=" + connection_url])
        MySoundProcessingModule = SoundProcessingModule(app)
        app.session.registerService("SoundProcessingModule", MySoundProcessingModule)
        MySoundProcessingModule.startProcessing()

        # Use rospy.spin() to keep the script alive until interrupted
        rospy.spin()

    except KeyboardInterrupt:
        # Unsubscribe and unregister the service
        MySoundProcessingModule.audio_service.unsubscribe(MySoundProcessingModule.module_name)
        app.session.unregisterService("SoundProcessingModule")

        app.stop()
        sys.exit(0)
