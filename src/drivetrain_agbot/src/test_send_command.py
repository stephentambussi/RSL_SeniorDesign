#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist
import sys


def send_data():
	#pub = rospy.Publisher('command_du_one', Int64, queue_size=10)
	#rospy.init_node('test_command_send', anonymous=True)
	#rate = rospy.Rate(10) # 10hz

	rospy.init_node("stopsign_commander")
	movement_publisher= rospy.Publisher('cmd_vel', Twist , queue_size=10)
	movement_cmd = Twist()

	while not rospy.is_shutdown():
		#hello_str = 311
		#pub.publish(hello_str)


		movement_cmd.linear.x = 10
		movement_publisher.publish(movement_cmd)		
		#print ("Publishing")
		#rospy.spin()      

		#rate.sleep()

if __name__ == '__main__':
    try:
        send_data()
    except rospy.ROSInterruptException:
        pass
