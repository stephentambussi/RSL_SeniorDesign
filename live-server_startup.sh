#!/bin/bash
#This will start up the interface for the robot
source /opt/ros/melodic/setup.bash
source /home/central-station/senior_design_ws/devel/setup.bash
#cd /home/central-station/senior_design_ws/src/tablet-webpage

#gnome-terminal --working-directory=/home/central-station/senior_design_ws/src/tablet-webpage -- live-server --port=5500
gnome-terminal -- /bin/bash -c "cd /home/central-station/senior_design_ws/src/tablet-webpage;/home/central-station/.nvm/versions/node/v14.4.0/bin/live-server --port=5500"