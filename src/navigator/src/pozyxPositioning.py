#!/usr/bin/env python
from std_msgs.msg import Float32MultiArray 

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

coords = Float32MultiArray()
coords.data = [0.0, 0.0] #initialize
x_avg = 0.0
y_avg = 0.0
x_cnt = 0
y_cnt = 0
pub_cnt = 0

#TODO: change avg calc to std dev calc to fix pozyx being jumpy

def findPozyxValues():
    data = serial_arduino.read_all()
    data = data.decode()
    data = data.split(",")
    data = data[1:3]
    global x_avg
    global y_avg
    global x_cnt
    global y_cnt
    global pub_cnt
    if len(data) == 2:
        x_str = data[0]
        y_str = data[1]
        x_str = x_str.split(": ")
        y_str = y_str.split(": ")
        #Take avg of last 5 values and publish them
        if "x" in x_str[0] and len(x_str) > 1:
            #coords.data[0] = float(x_str[1]) / 1000.0 #convert mm to meters
            x_avg = x_avg + float(x_str[1]) / 1000.0
            x_cnt = x_cnt + 1
        if "y" in y_str[0] and len(y_str) > 1:
            #coords.data[1] = float(y_str[1]) / 1000.0 #convert mm to meters
            y_avg = y_avg + float(y_str[1]) / 1000.0
            y_cnt = y_cnt + 1
    if x_cnt == 5:
        x_avg = x_avg / x_cnt #calculate average
        #TODO: change the round back to 3 decimal places after figuring out pozyx value filtering
        coords.data[0] = round(x_avg, 1)
        x_avg = 0
        x_cnt = 0
    if y_cnt == 5:
        y_avg = y_avg / y_cnt
        coords.data[1] = round(y_avg, 1)
        y_avg = 0
        y_cnt = 0

    if pub_cnt == 5: #this is to prevent repeat values from being published
        print(coords.data)
        pub.publish(coords)
        pub_cnt = 0
    pub_cnt = pub_cnt + 1

def sendPozyxValues():
    rospy.init_node('pozyx')
    global pub
    pub = rospy.Publisher('coordinates', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(25) #hz -- will publish pozyx values 5 times a second due to avg calculation
    time.sleep(1) #wait for pozyx to start up 
    while not rospy.is_shutdown():
        findPozyxValues()
        rate.sleep()

if __name__ == '__main__':
    sendPozyxValues()