#! /usr/bin/env python

import rospy
from std_msgs.msg import String

import serial

ser = serial.Serial("/dev/ttyACM0", 9600)

def callback(data):
    #b = bytes(data.data)
    ser.write(data.data)


rospy.init_node('ack_notifier_node')

rate = rospy.Rate(10) #10 Hz

if __name__ == "__main__":

    try:
        rospy.Subscriber('/icarus_driver/ack_notify', String, callback)
        rospy.spin()

    except KeyboardInterrupt:
        pass;
