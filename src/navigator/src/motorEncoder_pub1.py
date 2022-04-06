#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray

import rospy
import time
import serial

#This program will publish the actual RPM values for the wheels of encoder 1

#These are initializing the motor encoder USB port connections
ser_drive_unit_1 = serial.Serial(
port='/dev/serial/by-path/platform-3610000.xhci-usb-0:2.4:1.0',
baudrate=115200,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1
)

def readRPMs():
    enc_query1 = "?S 1\r"
    enc_query2 = "?S 2\r"
    #Continually ask for and publish RPM values
    try:
        while True:
            enc1_rpms = Int32MultiArray()
            enc1_rpms.data = [0, 0] #Initialize to zeroes

            #Send read RPM command to encoder 1
            ser_drive_unit_1.write(enc_query1) #Motor 1 of encoder 1
            rpm1 = ser_drive_unit_1.read_until(b'\r')
            rpm1 = rpm1.decode() #Convert byte to string
            #parse string
            if(rpm1.startswith("S=")):
                rpm1 = rpm1.split("=")
                rpm1 = rpm1[1]
                try:
                    rpm1 = int(rpm1) #Convert string to int
                    enc1_rpms.data[0] = rpm1
                except ValueError:
                    pass #ignore bad values
                #print("M1 RPM: ", rpm1)
            
            ser_drive_unit_1.write(enc_query2) #Motor 2 of encoder 1
            rpm2 = ser_drive_unit_1.read_until(b'\r')
            rpm2 = rpm2.decode() #Convert byte to string
            #parse string
            if(rpm2.startswith("S=")):
                rpm2 = rpm2.split("=")
                rpm2 = rpm2[1]
                try:
                    rpm2 = int(rpm2)
                    enc1_rpms.data[1] = rpm2
                except ValueError:
                    pass
                #print("M2 RPM: ", rpm2)

            #print("ENC1: ", enc1_rpms)
            pub1.publish(enc1_rpms)

    except KeyboardInterrupt:
        raise Exception("Ctrl-C pressed to kill program")

#Initialize everything
def wheelEncoder_pub():
    rospy.init_node('motorEncoder_pub1')

    global pub1
    pub1 = rospy.Publisher('encoder1_RPMs', Int32MultiArray, queue_size=10)
    
    readRPMs()

if __name__ == '__main__':
    wheelEncoder_pub()