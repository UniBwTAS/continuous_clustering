#!/usr/bin/env sh

if [ ! -f "/etc/localtime" ]
then
  echo 'Etc/UTC' > /etc/timezone && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime
fi

sudo apt update
sudo apt install -y -q --no-install-recommends curl tzdata dirmngr gnupg2 ca-certificates

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y -q --no-install-recommends ros-noetic-ros-core

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc