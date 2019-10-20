#! /usr/bin/env python

import rospy
from std_msgs.msg import String

import serial

class Notifier():
    def __init__(self):
        ser_ = None

        self.init_params()

        rospy.Subscriber('/icarus_driver/ack_notify', String, self.callback)

    def init_params(self):

        port = rospy.get_param('arduino_port', "/dev/ttyACM0")
        baudrate = rospy.get_param('baudrate', "9600")

        self.ser_ = serial.Serial(port, baudrate)

    def callback(self, data):
        #b = bytes(data.data)
        self.ser_.write(data.data)


rospy.init_node('ack_notifier_node')

rate = rospy.Rate(10) #10 Hz

if __name__ == "__main__":

    try:

        notifier = Notifier()
        rospy.spin()

    except KeyboardInterrupt:
        pass;
