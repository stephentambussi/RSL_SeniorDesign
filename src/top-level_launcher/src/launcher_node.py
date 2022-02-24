#!/usr/bin/env python

#TODO: create launch file for this node
#TODO: modify top-level_launcher_WIP.launch file to only launch top-level_launcher node
#TODO: Get Dylan to finish his portion and integrate into this one

import rospy
from std_msgs.msg import String #import something else later
import roslaunch

def launcher_callback(): #this method will receive input from tablet interface

    if(manual): # if manual mode selected
        rospy.init_node('name of node')
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, false)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["path to .launch file"])

        launch.start()
        rospy.loginfo("started")

    if(shutdown): # if person shutdown mode
        launch.shutdown()
