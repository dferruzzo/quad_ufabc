#!/bin/bash -i
# run this file at start up
# bash startup.sh
# neet to restart the terminal after running this file.
source /opt/ros/noetic/setup.bash
cd /home/catkin_ws
catkin_make
echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

source /usr/lib/git-core/git-sh-prompt
echo "PS1='${debian_chroot:+($debian_chroot)} \[\033[36m\]\u\[\033[37m\]@\[\033[33m\]\h: \[\033[32m\]\w\[\033[31;1m\]$(__git_ps1 " (%s)")\[\033[00m\]\$ '" >> ~/.bashrc

source ~/.bashrc
#cd ~/gzweb/http/client/assets/
#ln -s /home/catkin_ws/src/quad_ufabc/ ~/gzweb/http/client/assets/quad_ufabc
#roscd quad_ufabc
