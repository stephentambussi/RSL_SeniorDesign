#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray  # By Dhaval Patel

import rospy
import time
import serial


ser_drive_unit_1 = serial.Serial(
#port='/dev/serial/by-path/platform-3610000.xhci-usb-0:2.3:1.0',
port='/dev/serial/by-path/platform-3610000.xhci-usb-0:2.4:1.0',
baudrate=115200,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1
)

value_received = 0


def DriveUnit_1(val):

	print("Motor 1", val[0], "Motor 2", val[1],"Motor 3", val[2], "Motor 4", val[3])
	payload1 = "!G 1 " + str(round(val[0])) + "\r"  # chaged val[0] to 53
	payload2 = "!G 2 " + str(round(val[1])) + "\r"
	ser_drive_unit_1.write(payload1)
	ser_drive_unit_1.write(payload2)

def callback(data):
#	rospy.loginfo(data.data)
	value_received = data.data
#	print(value_received)
	DriveUnit_1(value_received)

def command_motors():
	rospy.init_node('command_du_one', anonymous=True)

	rospy.Subscriber("command_du_one", Int16MultiArray, callback)
	rospy.spin()

if __name__ == '__main__':
    command_motors()
