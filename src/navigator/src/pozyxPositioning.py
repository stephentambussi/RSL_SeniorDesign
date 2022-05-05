#!/usr/bin/env python
from msilib.schema import PublishComponent
from std_msgs.msg import Int16MultiArray, Int32MultiArray 

import rospy
import time
import serial


serial_arduino = serial.Serial(
#port='/dev/serial/by-path/platform-3610000.xhci-usb-0:2.3:1.0',
port='/dev/serial/by-path/platform-3610000.xhci-usb-0:2.2.4:1.0',
baudrate=115200,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1
)


def findPozyxValues(val):
    #Find and publish coordinates
    #Turn into Int32MultiArray
    #3D coordinates?


def callback(data):
    value_received = data.data
    findPozyxValues(value_received)

def sendPozyxValues():
    global pub
    pub = rospy.Publisher(#types of values published)
    rospy.Subscriber("xy_coords",Int16MultiArray,callback)
    pub.publish(#Values)
    rospy.spin()


    def start():
    rospy.init_node('Joy2Turtle')        # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    pub = rospy.Publisher('command_du_one', Int16MultiArray, queue_size=10)
                # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("zed_vel1", Float32MultiArray, callback)
                # starts the node
    
    rospy.spin()


if __name__ == '__main__':
    sendPozyxValues()

#count = 0
#for n in rpm:
#    if(n[0] == "S"):
#        ret[count] = n[2:]
#        count+=1