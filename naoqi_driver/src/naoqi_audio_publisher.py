#!/usr/bin/env python

import qi
import argparse
import sys
import time
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray

class SoundProcessingModule(object):
    def __init__(self, app, topic_name="/naoqi_driver/audio_publisher"):
        super(SoundProcessingModule, self).__init__()
        app.start()
        session = app.session
        self.audio_service = session.service("ALAudioDevice")  # Initialize audio service
        self.module_name = "SoundProcessingModule"
        self.micFront = []

        # ROS setup
        rospy.init_node('naoqi_audio_publisher', anonymous=True)
        self.pub = rospy.Publisher(topic_name, Int16MultiArray, queue_size=10)

    def startProcessing(self):
        self.audio_service.setClientPreferences(self.module_name, 16000, 3, 0)
        self.audio_service.subscribe(self.module_name)

        while not rospy.is_shutdown():
            time.sleep(1)

        self.audio_service.unsubscribe(self.module_name)

    def processRemote(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        self.micFront = np.frombuffer(inputBuffer, dtype=np.int16)

        # Publish the audio data to ROS
        audio_msg = Int16MultiArray(data=self.micFront.tolist())
        self.pub.publish(audio_msg)

if __name__ == "__main__":
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
