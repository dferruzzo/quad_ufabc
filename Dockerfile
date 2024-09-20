FROM dferruzzo/ros-noetic-gazebo-gzweb:v1.02

LABEL maintainer='diego.ferruzzo'
LABEL version='1.03'
LABEL description='Quadrotor in Ros Gazebo'

RUN apt -y update && apt install -y git python3-pip libqt5gui5 ros-noetic-ros-control ros-noetic-ros-controllers \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential exuberant-ctags \
    && rm -rf /var/lib/apt/lists/* 

RUN mkdir -p /home/catkin_ws/src/

WORKDIR /home/catkin_ws/src

