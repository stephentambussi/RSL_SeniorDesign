#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64

import rospy
import time
import serial


ser_drive_unit_1 = serial.Serial(
port='/dev/ttyACM0',
baudrate=115200,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1
)

def CH_1(val):
        payload1 = "!G 1 " + str(val) + "_"
        ser_drive_unit_1.write(payload1)

def CH_2(val):
        payload1 = "!G 2 " + str(val) + "_"
        ser_drive_unit_1.write(payload1)
 

value_received = 0

def callback1(data):
	value_received = data.data
	CH_1(value_received)

def callback2(data):
	value_received = data.data
	CH_2(value_received)


def command_motors():
	rospy.init_node('roboteq', anonymous=True)

	rospy.Subscriber("ch1", Int64, callback1)
	rospy.Subscriber("ch2", Int64, callback2)
	rospy.spin()

if __name__ == '__main__':
    command_motors()
