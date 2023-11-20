#!/usr/bin/env bash

mkdir -p kitti_odometry
cd kitti_odometry

wget -c https://s3.eu-central-1.amazonaws.com/avg-kitti/devkit_odometry.zip
# wget -c https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_gray.zip
# wget -c https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_color.zip
wget -c https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_velodyne.zip
wget -c https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_calib.zip
wget -c http://www.semantic-kitti.org/assets/data_odometry_labels.zip

unzip "*.zip"