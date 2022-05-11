#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float32MultiArray
from geometry_msgs.msg import Twist

go_where = 500 #initialize to an invalid value
x_coord = -1000.0 #initialize to an invalid value
y_coord = -1000.0
init_x = -1000.0
init_y = -1000.0

def goto_callback(msg):
    global go_where

    curr_go = msg.data
    #print(curr_go)

    go_where = curr_go

def pozyx_callback(float):
    global x_coord
    global y_coord

    print(float.data)
    x_coord = float.data[0]
    y_coord = float.data[1]

def do_goto():
    #TODO: fix code getting stuck in an infinite loop when driving forward
    #TODO: implement the rest of the go to buttons
    global init_x
    init_x = 0 #clear value
    global init_y
    init_y = 0
    global go_where
    global x_coord
    global y_coord
    if go_where == 10:
        move_cmd = Twist()
        move_cmd.linear.x = 0.2 #m/s
        move_cmd.linear.y = 0.0
        move_cmd.angular.z = 0.0 #rad/s
        init_x = x_coord
        init_y = y_coord
        goal = init_x + 1
        while x_coord <= goal:
            pub.publish(move_cmd)
            print(x_coord)
        go_where = 500 #reset go_where so function does not keep running

def start():
    rospy.init_node("goTo")

    global pub
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    rospy.Subscriber("go_to", Int16, goto_callback)
    rospy.Subscriber("coordinates", Float32MultiArray, pozyx_callback)
    rate = rospy.Rate(5) #hz
    while not rospy.is_shutdown():
        do_goto()
        rate.sleep()

if __name__ == "__main__":
    start()