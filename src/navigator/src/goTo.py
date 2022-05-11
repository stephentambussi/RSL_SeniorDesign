#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16, Float32MultiArray
from geometry_msgs.msg import Twist
import time

go_where = 500 #initialize to an invalid value
x_coord = -1000.0 #initialize to an invalid value
y_coord = -1000.0
init_x = -1000.0
init_y = -1000.0

#TODO: get heading from ZED IMU
def goto_callback(msg):
    global go_where

    curr_go = msg.data
    #print(curr_go)

    go_where = curr_go

def pozyx_callback(float):
    global x_coord
    global y_coord

    #print(float.data)
    x_coord = round(float.data[0], 2)
    y_coord = round(float.data[1], 2)

def do_goto(move_cmd):
    global x_coord
    global y_coord
    elapsed = 0 #elapsed time
    start = time.time()
    while elapsed < 5:
        pub.publish(move_cmd)
        print(x_coord)
        end = time.time()
        elapsed = end - start

def start():
    rospy.init_node("goTo")

    global pub
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    global go_where
    global init_x
    global init_y
    global x_coord
    global y_coord
    rospy.Subscriber("go_to", Int16, goto_callback)
    rospy.Subscriber("coordinates", Float32MultiArray, pozyx_callback)
    rate = rospy.Rate(5) #hz
    while not rospy.is_shutdown():
        if go_where == 10: #go forward by a meter
            move_cmd = Twist()
            move_cmd.linear.x = 0.2 #m/s
            move_cmd.linear.y = 0.0
            move_cmd.angular.z = 0.0 #rad/s
            init_x = x_coord
            init_y = y_coord
            init_str = "Initial Position: x = " + str(init_x) + " y = " + str(init_y)
            print(init_str)
            do_goto(move_cmd)
            go_where = 500 #reset go_where so function does not keep running
        elif go_where == 11: #go backward by a meter
            move_cmd = Twist()
            move_cmd.linear.x = -0.2 #m/s
            move_cmd.linear.y = 0.0
            move_cmd.angular.z = 0.0 #rad/s
            init_x = x_coord
            init_y = y_coord
            init_str = "Initial Position: x = " + str(init_x) + " y = " + str(init_y)
            print(init_str)
            do_goto(move_cmd)
            go_where = 500 #reset go_where so function does not keep running
        elif go_where == 12: #go left by a meter
            move_cmd = Twist()
            move_cmd.linear.x = 0.0 #m/s
            move_cmd.linear.y = -0.2
            move_cmd.angular.z = 0.0 #rad/s
            init_x = x_coord
            init_y = y_coord
            init_str = "Initial Position: x = " + str(init_x) + " y = " + str(init_y)
            print(init_str)
            do_goto(move_cmd)
            go_where = 500 #reset go_where so function does not keep running
        elif go_where == 13: #go right by a meter
            move_cmd = Twist()
            move_cmd.linear.x = 0.0 #m/s
            move_cmd.linear.y = 0.2
            move_cmd.angular.z = 0.0 #rad/s
            init_x = x_coord
            init_y = y_coord
            init_str = "Initial Position: x = " + str(init_x) + " y = " + str(init_y)
            print(init_str)
            do_goto(move_cmd)
            go_where = 500 #reset go_where so function does not keep running
        rate.sleep()

if __name__ == "__main__":
    start()