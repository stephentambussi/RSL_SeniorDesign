#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray

import rospy
import time
import serial

#This program will publish the actual RPM values for the wheels of encoder 2

#These are initializing the motor encoder USB port connections
ser_drive_unit_2 = serial.Serial(
port='/dev/serial/by-path/platform-3610000.xhci-usb-0:2.3:1.0',
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
            enc2_rpms = Int32MultiArray()
            enc2_rpms.data = [0, 0] #Initialize to zeros

            #Send read RPM command to encoder 2
            ser_drive_unit_2.write(enc_query1) #Motor 1 of encoder 2
            rpm3 = ser_drive_unit_2.read_until(b'\r')
            rpm3 = rpm3.decode() #Convert byte to string
            #parse string
            if(rpm3.startswith("S=")):
                rpm3 = rpm3.split("=")
                rpm3 = rpm3[1]
                try:
                    rpm3 = int(rpm3)
                    enc2_rpms.data[0] = rpm3
                except ValueError:
                    pass
                #print("M3 RPM: ", rpm3)

            ser_drive_unit_2.write(enc_query2) #Motor 2 of encoder 2
            rpm4 = ser_drive_unit_2.read_until(b'\r')
            rpm4 = rpm4.decode() #Convert byte to string
            #parse string
            if(rpm4.startswith("S=")):
                rpm4 = rpm4.split("=")
                rpm4 = rpm4[1]
                try:
                    rpm4 = int(rpm4)
                    enc2_rpms.data[1] = rpm4
                except ValueError:
                    pass
                #print("M4 RPM: ", rpm4)

            #print("ENC2: ", enc2_rpms)
            pub2.publish(enc2_rpms)

    except KeyboardInterrupt:
        raise Exception("Ctrl-C pressed to kill program")

#Initialize everything
def wheelEncoder_pub():
    rospy.init_node('motorEncoder_pub2')
    
    global pub2
    pub2 = rospy.Publisher('encoder2_RPMs', Int32MultiArray, queue_size=10)
    readRPMs()

if __name__ == '__main__':
    wheelEncoder_pub()