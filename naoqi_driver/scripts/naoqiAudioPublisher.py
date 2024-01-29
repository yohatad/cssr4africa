#!/usr/bin/env python3

import socket
import rospy
import numpy as np
from naoqi_driver.msg import AudioCustomMsg

def udp_ros_client():
    rospy.init_node('NaoqiAudioPublisher')
    pub = rospy.Publisher('/naoqi_audio', AudioCustomMsg, queue_size=10)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(('0.0.0.0', 9999))

    nbOfSamplesByChannel = 4096

    while not rospy.is_shutdown():
        data, addr = udp_socket.recvfrom(32768)  # Adjust buffer size as needed
        audio = np.frombuffer(data, dtype=np.int16)

        micLeft = audio[0:nbOfSamplesByChannel].tolist()
        micRight = audio[nbOfSamplesByChannel:nbOfSamplesByChannel*2].tolist()
        micFront = audio[nbOfSamplesByChannel*2:nbOfSamplesByChannel*3].tolist()
        micRear = audio[nbOfSamplesByChannel*3:nbOfSamplesByChannel*4].tolist()

        msg = AudioCustomMsg()
        msg.micLeft = micLeft
        msg.micRight = micRight
        msg.micFront = micFront
        msg.micRear = micRear
        pub.publish(msg)
                    
    udp_socket.close()

if __name__ == "__main__":
    udp_ros_client()