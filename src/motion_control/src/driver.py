#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import Int16MultiArray, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np

    # Author: Dhaval Patel
    #copyright=  Robotics Systems Laboratory, Santa Clara University
    
def callback(Twist):
    #pub = rospy.Publisher('command_du_one', Int16MultiArray, queue_size=10)
    motors = Int16MultiArray()
    arr = np.array([Twist.linear.x, Twist.linear.y, Twist.angular.z])
     
    motors = Int16MultiArray()
    pi = 3.14
    velocities = np.array([[Twist.linear.x], [Twist.linear.y], [Twist.angular.z]]) # taken from joystick in m/s
    vel_str = "Driver Node Velocities: x = " + str(Twist.linear.x) + " y = " + str(Twist.linear.y) + " theta = " + str(Twist.angular.z)
    print(vel_str)
    #----------defining wheel rotations based on velocites recieved for holonomic motion-------#
    
    # declaring robot wheel radius and distances of the wheels along x and y as l1 and l2
    R =  0.0635 #  Taking wheel radius in meters   
    l1 = 0.3048  # Taking the distance of the wheels in +x and -x directions 
    l2 = 0.3048 #   "     "                             in +y and -y directions
    Transformation = np.array([[1,  -1,  -(l1+l2)], # Matrix to transform linear velocites to wheel 
                               [1, 1,   (l1+l2)], # rotations in rad/s
                               [1, 1,  -(l1+l2)],
                               [1,  -1,   (l1+l2)]])
    Angular_velocities = (1/R)*np.matmul(Transformation,velocities)
    RPMs = (60.0/(2*pi))*Angular_velocities 
    
    motors.data = [round(RPMs[0]),round(RPMs[1]),round(RPMs[2]),round(RPMs[3])]
    #print(motors.data)
    global pub
    pub.publish(motors)

def temp_callback(String):
    print(String.data)

##    


    # Intializes everything
def start():
    rospy.init_node('Joy2Turtle')        # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    pub = rospy.Publisher('command_du_one', Int16MultiArray, queue_size=10)
                # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("cmd_vel", Twist, callback)
                # starts the node
    rospy.Subscriber("read", String, temp_callback)
    
    rospy.spin()

if __name__ == '__main__':
    start()



##import rospy

##from sensor_msgs.msg import Joy
##import numpy as np
##
##def send_data(joy_msg):
##    global pub
##    
##    
##    
##    rate = rospy.Rate(10) # 10hz
##    
##    motors = Int16MultiArray()
####  arr = np.array(joy_msg.axes) 
####  arr = arr*300
####  motors.data[0] = int(arr[4] - arr[3])
####  motors.data[1] = int(arr[4] + arr[3])
##    print("Motors data computed and sent  = ",joy_msg.axes)
##        #pub.publish(motors)
##    rate.sleep()
##
##def start():
##    rospy.init_node('test_command_send', anonymous=True)
##    rospy.Rate(10)
##    global pub
##    pub = rospy.Publisher('command_du_one', Int16MultiArray, queue_size=10)
##    rospy.init_node('test_command_send', anonymous=True)
##    rospy.Subscriber("joy",Joy,send_data)
##    rospy.spin()
##    
##if __name__ == '__main__':
##    try:
##        start()
##    except rospy.ROSInterruptException:
##        pass
