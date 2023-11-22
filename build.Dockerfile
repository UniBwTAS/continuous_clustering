FROM osrf/ros:noetic-desktop-focal

ENV HOME /root
ENV DEBIAN_FRONTEND noninteractive

# copy scripts into docker image
COPY scripts/* /usr/local/bin/

# install build tools and create ROS workspace
RUN setup_workspace.sh && rm -rf /var/lib/apt/lists/*

# install dependencies and build workspace
WORKDIR ${HOME}/catkin_ws/src
COPY . ./continuous_clustering
RUN install_dependencies_and_build.sh --from-source && rm -rf /var/lib/apt/lists/* && rm -rf ${HOME}/.ros

