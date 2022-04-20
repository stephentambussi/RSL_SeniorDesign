#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray, Int32MultiArray  # By Dhaval Patel

import rospy
import time
import serial


ser_drive_unit_2 = serial.Serial(
port='/dev/serial/by-path/platform-3610000.xhci-usb-0:2.3:1.0',
baudrate=115200,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1
)

value_received = 0

def DriveUnit_2(val):
	enc_query = "?S 1_?S 2_"
	enc2_rpms = Int32MultiArray()
	enc2_rpms.data = [0, 0]

	# print("Motor 1", val[0],"Motor 2", val[1],"Motor 3", val[2],"Motor 4", val[3])

	payload3 = "!G 1 " + str(val[2]) + "_"
	payload4 = "!G 2 " + str(val[3]) + "_"  # change this to test

	ser_drive_unit_2.write(payload3)
	ser_drive_unit_2.write(payload4)

	#Publish RPMs of motor 4 (right side) for odom
	ser_drive_unit_2.write(enc_query) #Motor 1,2 of encoder 2
	rpm4 = ser_drive_unit_2.read_all() #get motors of encoder 2
	rpm4 = rpm4.decode() #Convert byte to string
	rpm4 = rpm4.split("\r")
	#print("Encoder 2: ", rpm4)
	#parse string
	substr = "S="
	count=0
	for string in rpm4:
		if string != None and string.startswith(substr) and count < 2:
			enc2_rpms.data[count] = int(string[2:])
			count += 1
	if count != 0:
		#print(enc2_rpms.data)
		pub.publish(enc2_rpms)

def callback(data):
#	rospy.loginfo(data.data)
	value_received = data.data
#	print(value_received)
	DriveUnit_2(value_received)


def command_motors():
	rospy.init_node('command_du_one', anonymous=True)

	global pub
	pub = rospy.Publisher('enc2_RPMs', Int32MultiArray, queue_size=10)

	rospy.Subscriber("command_du_one", Int16MultiArray, callback)
	rospy.spin()

if __name__ == '__main__':
    command_motors()
