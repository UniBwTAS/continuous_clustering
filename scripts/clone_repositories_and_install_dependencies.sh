#!/usr/bin/env sh

# install git in order to clone all dependencies
sudo apt update
sudo apt install -y -q git

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
if [ "$1" != "--from-source" ]
then
  git clone https://github.com/UniBwTAS/continuous_clustering.git
fi

# source ROS
. /opt/ros/noetic/setup.sh

# install remaining dependencies in package.xml's from apt & pip repositories
sudo apt install -y -q python3-rosdep
sudo rosdep init || true
rosdep update
rosdep install --from-paths . --ignore-src -r -y