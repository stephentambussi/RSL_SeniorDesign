#!/usr/bin/env python
import rospy
from std_msgs.msgs import Int16

import roslaunch

active_mode = 5 #initialize to 5 which is not any valid mode

def mode_callback(Int16):
    #TODO: see if you need to add functionality where previous node is shutdown specifically when mode switches
    #or if you can just do launch.shutdown() at the beginning of each mode
    global active_mode
    global launch

    current_mode = Int16.data

    if current_mode == 0: #manual mode
        active_mode = 0 #TEST
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/central-station/senior_design_ws/src/motion_control/launch/manual_control.launch"])
        launch.start()
        rospy.loginfo("Started Manual Mode")
    
    elif current_mode == 1: #auto mode
        active_mode = 1
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/central-station/senior_design_ws/src/omnibot_2dnav/launch/omnibot.launch"])
        launch.start()
        rospy.loginfo("Started Auto Mode")
    
    elif current_mode == 2: #follow mode
        active_mode = 2
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/central-station/senior_design_ws/src/navigator/launch/follow_me.launch"])
        launch.start()
        rospy.loginfo("Started Follow Mode")
    
    elif current_mode == -1: #stop robot
        active_mode = -1
        #TODO: check this
        try:
            launch.shutdown()
        except:
            print("There is not a mode active")
        else:
            launch.shutdown() #for redundancy




def main():
    rospy.init_node('mode_switcher', anonymous=True)

    global uuid
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    rospy.Subscriber("start_mode", Int16, mode_callback)
    rospy.spin()

if __name__ == "__main__":
    main()