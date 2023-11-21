#!/usr/bin/env bash

# source ROS installation
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# clone all dependencies
sudo apt install -y git

# library to do continuous clustering with ouster sensor
git clone --recursive https://github.com/ouster-lidar/ouster-ros.git

# library to do continuous clustering for velodyne sensor (with recorded UDP Packets via ethernet_bridge)
git clone https://github.com/1r0b1n0/librosqt.git
git clone https://github.com/UniBwTAS/ethernet_bridge.git

# visualization plugins (+ dependencies) for rviz
git clone https://github.com/UniBwTAS/ogre_helpers.git
git clone https://github.com/UniBwTAS/rviz_range_image.git
git clone https://github.com/UniBwTAS/rviz_continuous_point_cloud.git
git clone https://github.com/UniBwTAS/rviz_colorize_point_cloud_by_label.git

# actual continuous clustering package
git clone https://github.com/UniBwTAS/continuous_clustering.git

# install remaining dependencies in package.xml's from apt repository
sudo apt install -y python3-rosdep
sudo rosdep init || true
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# build & source directory
catkin build
source ~/catkin_ws/devel/setup.bash