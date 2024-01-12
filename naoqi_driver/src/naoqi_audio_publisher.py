#!/usr/bin/env python

import qi
import argparse
import sys
import time
import numpy as np
import rospy
from audio_common_msgs.msg import AudioData  # Import AudioData from audio_common_msgs

class SoundProcessingModule(object):
    def __init__(self, app, topic_name="/naoqi_driver/audio"):
        super(SoundProcessingModule, self).__init__()
        app.start()
        session = app.session
        self.audio_service = session.service("ALAudioDevice")  # Initialize audio service
        self.module_name = "SoundProcessingModule"
        self.micFront = []

        # ROS setup with AudioData message type
        rospy.init_node('naoqi_audio_publisher', anonymous=True)
        self.pub = rospy.Publisher(topic_name, AudioData, queue_size=10)

    def startProcessing(self):
        self.audio_service.setClientPreferences(self.module_name, 16000, 3, 0)
        self.audio_service.subscribe(self.module_name)

        while not rospy.is_shutdown():
            time.sleep(1)

        self.audio_service.unsubscribe(self.module_name)

    def processRemote(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        self.micFront = np.frombuffer(inputBuffer, dtype=np.int16)

        # Convert the numpy array to a byte array and publish it
        audio_data = AudioData()
        audio_data.data = self.micFront.tobytes()
        self.pub.publish(audio_data)

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('naoqi_audio_publisher', anonymous=True)

    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="172.29.111.232", help="Robot IP address.")
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
