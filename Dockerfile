FROM dferruzzo/ros-noetic-gazebo-gzweb:v1.02

LABEL maintainer='diego.ferruzzo'
LABEL version='1.03'
LABEL description='Quadrotor in Ros Gazebo'

# Fix ROS repository signing issues and update sources
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release ca-certificates \
    && rm -f /etc/apt/sources.list.d/ros-latest.list \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros-latest.list > /dev/null \
    && apt-get update --allow-unauthenticated || apt-get update

RUN apt -y update && apt install -y git python3-pip libqt5gui5 ros-noetic-ros-control ros-noetic-ros-controllers \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential exuberant-ctags \
    && rm -rf /var/lib/apt/lists/* 

RUN mkdir -p /home/catkin_ws/src/

WORKDIR /home/catkin_ws/src

