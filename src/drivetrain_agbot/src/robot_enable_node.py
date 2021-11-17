#! /usr/bin/env python
# publisher: read deadman switch, and enable the robot

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16

def callback(data):

        deadman_button_state = 1 - data.buttons[4]
#       print(deadman_button_state)

        pub = rospy.Publisher('robot_enable', UInt16, queue_size=10)
        pub.publish(deadman_button_state)



def start():
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, callback)
        # starts the node
        rospy.init_node('robot_enable_node')
        rospy.spin()

if __name__ == '__main__':
        start()


