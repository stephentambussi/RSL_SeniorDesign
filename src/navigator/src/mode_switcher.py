#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

import roslaunch

active_mode = 5 #initialize to 5 which is not any valid mode

def mode_callback(msg):
    global active_mode

    current_mode = msg.data
    print(current_mode)

    if current_mode == 0: #manual mode
        active_mode = 0 
    
    elif current_mode == 1: #auto mode
        active_mode = 1
    
    elif current_mode == 2: #follow mode
        active_mode = 2
    
    elif current_mode == -1: #stop robot
        active_mode = -1

def start():
    rospy.init_node('mode_switcher')

    global uuid
    global launch
    running_mode = 0 #boolean to see if a mode is currently running
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    rospy.Subscriber("start_mode", Int16, mode_callback)
    #rospy.spin()

    while not rospy.is_shutdown():
        if running_mode == 0:
            if active_mode == 0: #manual mode
                launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/central-station/senior_design_ws/src/motion_control/launch/manual_control.launch"])
                launch.start()
                rospy.loginfo("Started Manual Mode")
                print("manual mode test")
                running_mode = 1
            elif active_mode == 1: #auto mode
                launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/central-station/senior_design_ws/src/omnibot_2dnav/launch/omnibot.launch"])
                launch.start()
                rospy.loginfo("Started Auto Mode")
                print("auto mode test")
                running_mode = 1
            elif active_mode == 2: #follow mode
                launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/central-station/senior_design_ws/src/navigator/launch/follow_me.launch"])
                launch.start()
                rospy.loginfo("Started Follow Mode")
                print("follow mode test")
                running_mode = 1
        if running_mode == 1 and active_mode == -1: #shutdown currently running mode
            try:
                launch.shutdown()
                print("mode shutdown test")
                running_mode = 0
            except:
                print("There is not a mode active")
            #Delete this else if it becomes unnecessary
            else:
                launch.shutdown() #for redundancy
                print("mode shutdown test")
                running_mode = 0

if __name__ == "__main__":
    start()