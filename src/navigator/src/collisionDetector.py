#!/usr/bin/env python
from std_msgs.msg import Int16
import serial
import rospy

port_1 = '/dev/serial/by-path/platform-3610000.xhci-usb-0:2.2.3:1.0'
port_2 = '/dev/ttyACM0'

serial_coll = serial.Serial(
port=port_1,
baudrate=9600,
timeout=10
)

def findCollisionSignal():
	data = serial_coll.read_all()
	signal = Int16()
	signal.data = 616
	#print(type(data))
	if(data.startswith("BL")):
		print("Back Left")
		pub.publish(signal)
	if(data.startswith("BR")):
		print("Back Right")
		pub.publish(signal)
	if(data.startswith("FL")):
		print("Front Left")
		pub.publish(signal)
	if(data.startswith("FR")):
		print("Front Right")
		pub.publish(signal)

def sendCollisionSignal():
	rospy.init_node('collisionDetector')
	global pub
	pub = rospy.Publisher('collisionSignal', Int16, queue_size=10)
	rate = rospy.Rate(5) #hz
	while not rospy.is_shutdown():
		findCollisionSignal()
		rate.sleep()

if __name__ == '__main__':
	sendCollisionSignal()