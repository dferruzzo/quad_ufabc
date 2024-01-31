#!/bin/bash -i
# run this file at start up
# bash startup.sh
# neet to restart the terminal after running this file.
source /opt/ros/noetic/setup.bash
cd /home/catkin_ws
catkin_make
echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc