#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray, Int32MultiArray  # By Dhaval Patel

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

enc1_rpms = Int32MultiArray()
enc1_rpms.data = [0, 0] #initialize

def DriveUnit_1(val):
	enc_query = "?S 1_?S 2_"

	#print("Motor 1", val[0], "Motor 2", val[1],"Motor 3", val[2], "Motor 4", val[3])
	payload1 = "!G 1 " + str(round(val[0])) + "_"  # chaged val[0] to 53
	payload2 = "!G 2 " + str(round(val[1])) + "_"

	ser_drive_unit_1.write(payload1)
	ser_drive_unit_1.write(payload2)

	#Publish RPMs of motor 1 (left side) for odom
	ser_drive_unit_1.write(enc_query) #Motor 1,2 of encoder 1
	rpm1 = ser_drive_unit_1.read_all() #get motors of encoder 1
	rpm1 = rpm1.decode() #Convert byte to string
	rpm1 = rpm1.split("\r")
	#print("Encoder 1: ", rpm1)
	#parse string
	substr = "S="
	count=0
	for string in rpm1:
		if string != None and string.startswith(substr) and count < 2:
			enc1_rpms.data[count] = int(string[2:])
			count += 1
	if count != 0:
		#print(enc1_rpms.data)
		publish_flag = 1

def callback(data):
#	rospy.loginfo(data.data)
	value_received = data.data
#	print(value_received)
	DriveUnit_1(value_received)

def command_motors():
	rospy.init_node('command_du_one', anonymous=True)
	rate = rospy.Rate(30) #hz

	global pub
	pub = rospy.Publisher('enc1_RPMs', Int32MultiArray, queue_size=10)

	rospy.Subscriber("command_du_one", Int16MultiArray, callback)

	#rospy.spin()

	while not rospy.is_shutdown():
		#TODO: test resetting enc1_rpms to 0 after publish
		#if publish_flag == 1:
		pub.publish(enc1_rpms)
		#publish_flag = 0
		rate.sleep()

if __name__ == '__main__':
    command_motors()
