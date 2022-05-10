#!/bin/bash
#This will start up the nodes for running everything
source /opt/ros/melodic/setup.bash
source /home/central-station/senior_design_ws/devel/setup.bash

gnome-terminal --working-directory=/home/central-station/senior_design_ws -- roslaunch rosbridge_server rosbridge_websocket.launch

sleep 5
gnome-terminal --working-directory=/home/central-station/senior_design_ws -- rosrun navigator mode_switcher.py
gnome-terminal --working-directory=/home/central-station/senior_design_ws -- rosrun web_video_server web_video_server
