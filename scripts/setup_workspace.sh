#!/usr/bin/env bash

# setup ROS workspace
sudo apt update
sudo apt install -y build-essential cmake python3-catkin-tools
source /opt/ros/noetic/setup.bash
cd ~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
cd src
catkin config --profile default --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc