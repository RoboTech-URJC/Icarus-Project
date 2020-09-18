#! /usr/bin/env python

import rospy
from std_msgs.msg import String

import serial
import os

# Global variable:

NodeName = 'ack_notifier_node'


class Notifier():
    def __init__(self):
        self.ser_ = None
        self.port_ = None

        self.init_params()

        rospy.Subscriber('/icarus_driver/ack_notify', String, self.callback)

    def init_params(self):

        self.port_ = rospy.get_param('~arduino_port', '/dev/ttyACM0')
        baudrate = rospy.get_param('~baudrate', '9600')
        try:
            self.ser_ = serial.Serial(self.port_, baudrate)
        except serial.serialutil.SerialException:
            self.raise_except()

    def callback(self, data):
        # b = bytes(data.data)

        self.ser_.write(data.data)

    def raise_except(self):
        rospy.logerr("[%s] Ligths Handler Serial Port could not be opened\n", self.port_)
        os.system("rosnode kill " + NodeName)

if __name__ == "__main__":

    rospy.init_node(NodeName)

    rate = rospy.Rate(10)   # 10 Hz
    try:

        notifier = Notifier()
        rospy.spin()

    except KeyboardInterrupt:
        pass
