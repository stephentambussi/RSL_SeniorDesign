#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Int16MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist

def goto_callback(msg):

def pozyx_callback(float):

def do_goto():


def start():
    rospy.init_node("goTo")

    global pub
    pub = rospy.Publisher("cmd_vel", Int16MultiArray, queue_size=10)

    rospy.Subscriber("go_to", Int16, goto_callback)
    rospy.Subscriber("coordinates", Float32MultiArray, pozyx_callback)
    rate = rospy.Rate(25) #hz
    while not rospy.is_shutdown():
        do_goto()
        rate.sleep()

if __name__ == "__main__":
    start()