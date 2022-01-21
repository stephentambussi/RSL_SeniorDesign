#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray  ### By Dhaval Patel

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

def DriveUnit_1(val):
	
	#print("Detected obstacle distance = ", val[0]/10.0,"meters")
        payload1 = "@02!G 1 " + str(round(val[0])) + "_"
	payload2 = "@02!G 2 " + str(round(val[1])) + "_"
	#payload3 = "!G 1 " + str(100) + "_"
        #payload4 = "!G 2 " + str(100) + "_"
        ser_drive_unit_1.write(payload1)
        ser_drive_unit_1.write(payload2)
	#ser_drive_unit_1.write(payload3)
        #ser_drive_unit_1.write(payload4)
	
	

value_received = 0

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
