FROM osrf/ros:noetic-desktop-full

LABEL maintainer='diego.ferruzzo'
LABEL version='1.0'
LABEL description='Ros-Gazebo envieronment for quadrotor UFABC testing'

RUN apt -y update && apt install -y git python3-pip libqt5gui5 ros-noetic-ros-control ros-noetic-ros-controllers \
                     python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
                     && rm -rf /var/lib/apt/lists/* 

ENV QT_DEBUG_PLUGINS=1

WORKDIR /workspace/src
