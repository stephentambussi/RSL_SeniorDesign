#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray  # By Dhaval Patel

import rospy
import time
import serial


ser_drive_unit_2 = serial.Serial(
port='/dev/serial/by-path/platform-3610000.xhci-usb-0:2.4:1.0',
baudrate=115200,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1
)

value_received = 0


def DriveUnit_2(val):

	# print("Motor 1", val[0],"Motor 2", val[1],"Motor 3", val[2],"Motor 4", val[3])

	payload3 = "!G 1 " + str(val[2]) + "_"
	payload4 = "!G 2 " + str(val[3]) + "_"  # change this to test

	ser_drive_unit_2.write(payload3)
	ser_drive_unit_2.write(payload4)

def callback(data):
#	rospy.loginfo(data.data)
	value_received = data.data
#	print(value_received)
	DriveUnit_2(value_received)


def command_motors():
	rospy.init_node('command_du_one', anonymous=True)

	rospy.Subscriber("command_du_one", Int16MultiArray, callback)
	rospy.spin()

if __name__ == '__main__':
    command_motors()
