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


def findPozyxValues():
    data = serial_arduino.read()
    whichnum = 0
    flag = 0
    nums = ["","","","",""]
    for char in data:
        if char.isdigit() or char == '-':
            nums[whichnum] = nums[whichnum] + char
            flag = 1
        else:
            if flag == 1:
                whichnum = whichnum + 1
            flag = 0
    if nums[4] != "":
        coords = [nums[2],nums[3]]
        return(coords)


def callback():
    coords = findPozyxValues()
    #switch coords to Float32MultiArray
    pub.publish(coords)

def sendPozyxValues():
    global pub
    pub = rospy.Publisher('coordinates',Float32MultiArray,queue_size=10)
    
    rospy.spin()


if __name__ == '__main__':
    sendPozyxValues()