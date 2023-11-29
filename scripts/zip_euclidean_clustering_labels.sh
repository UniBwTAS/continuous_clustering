#!/usr/bin/env sh

if [ -z $KITTI_SEQUENCES_PATH ];
then
  echo "Please set KITTI_SEQUENCES_PATH environment variable before executing this script"
  exit
fi

cd $KITTI_SEQUENCES_PATH
find . -type d -name "labels_euclidean_clustering" -exec mkdir -p /tmp/dataset/sequences/{} \; -exec cp -r {} /tmp/dataset/sequences/{}/.. \;
cd /tmp
zip -r data_odometry_labels_euclidean_clustering.zip dataset
rm -rf /tmp/dataset