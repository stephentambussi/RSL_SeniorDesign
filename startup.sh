#This will start up the robot for running everything
source /opt/ros/melodic/setup.bash
#source catkin_ws/devel/setup.bash
source /home/central-station/senior_design_ws/devel/setup.bash

gnome-terminal --working-directory=/home/central-station/senior_design_ws -- roslaunch rosbridge_server rosbridge_websocket.launch

sleep 5
gnome-terminal --working-directory=/home/central-station/senior_design_ws -- rosrun navigator mode_switcher.py
gnome-terminal --working-directory=/home/central-station/senior_design_ws -- rosrun web_video_server web_video_server
#gnome-terminal --working-directory=/home/central-station/senior_design_ws/src/tablet-webpage -- live-server --port=5500
cd /home/central-station/senior_design_ws/src/tablet-webpage
live-server --port=5500