[![Basic Build Workflow](https://github.com/UniBwTAS/continuous_clustering/actions/workflows/basic-build-ci.yaml/badge.svg?branch=master)](https://github.com/UniBwTAS/continuous_clustering/actions/workflows/basic-build-ci.yaml)

## Low Latency Instance Segmentation by Continuous Clustering for Rotating LiDAR Sensors

Low-latency instance segmentation of LiDAR point clouds is crucial in real-world applications because it serves as an initial and frequently-used building block in a robot's perception pipeline, where every task adds further delay.
Particularly in dynamic environments, this total delay can result in significant positional offsets of dynamic objects, as seen in highway scenarios.
To address this issue, we employ continuous clustering of obstacle points in order to obtain an instance-segmented point cloud.
Unlike most existing approaches, which use a full revolution of the LiDAR sensor, we process the data stream in a continuous and seamless fashion.
More specifically, each column of a range image is processed as soon it is available.
Obstacle points are clustered to existing instances in real-time and it is checked at a high-frequency which instances are completed and  are ready to be published.
An additional advantage is that no problematic discontinuities between the points of the start and the end of a scan are observed.
In this work we describe the two-layered data structure and the corresponding algorithm for continuous clustering, which is able to cluster the incoming data in real time.
We explain the importance of a large perceptive field of view.
Furthermore, we describe and evaluate important architectural design choices, which could be relevant to design an architecture for deep learning based low-latency instance segmentation.

![](https://github.com/UniBwTAS/continuous_clustering/blob/master/assets/demo.gif)

## Run it yourself:

### Download Sensor Data

```bash
# location where to download all the files
export SENSOR_DATA_PATH=/desired/folder/with/enough/space

# download small example rosbag recorded with our test vehicle (VW Touareg)
cd $SENSOR_DATA_PATH
mkdir -p rosbags && cd rosbags && curl -O https://todo-upload-rosbag.asdf/vw_touareg_example1.bag

# download kitti odometry dataset & semantic kitti labels (if not already done)
# this takes quite some time -> skip this step if rosbag is enough for you or you want to try it as fast as possible
cd $SENSOR_DATA_PATH
curl -s https://raw.githubusercontent.com/UniBwTAS/continuous_clustering/master/scripts/kitti_odometry_downloader.sh | bash -s
```
### Setup Environment
#### Option 1: Docker + GUI (VNC):

```bash
# install docker engine (e.g.: https://docs.docker.com/engine/install/ubuntu/)

# ensure shell variable SENSOR_DATA_PATH is (still) set correctly (see above)

# pull and run docker container
docker run -d -p 6080:80 -v /dev/shm:/dev/shm -v ${SENSOR_DATA_PATH}/kitti_odometry/dataset/sequences:/mnt/kitti_odometry/dataset/sequences -v ${SENSOR_DATA_PATH}/rosbags/:/mnt/rosbags -e SENSOR_DATA_PATH=/mnt  --name continuous_clustering_demo andreasr30/continuous_clustering_demo:focal-ros1-vnc

# 1. open your browser and enter: http://localhost:6080 (wait a few seconds and retry if it does not load)
# 2. open terminal in browser window (click 'Start' -> 'System Tools' -> 'LXTerminal')
# 3. continue with step "Run Continuous Clustering" in the terminal opened 2.
```

#### Option 2: Locally on Ubuntu 20.04 (Focal) and ROS Noetic

```bash
# install ROS (if not already installed)
wget -P /tmp https://raw.githubusercontent.com/UniBwTAS/continuous_clustering/master/scripts/install_ros.sh
bash /tmp/install_ros.sh

# setup ROS workspace (if not already existing)
wget -P /tmp https://raw.githubusercontent.com/UniBwTAS/continuous_clustering/master/scripts/setup_workspace.sh
bash /tmp/setup_workspace.sh # created at ~/catkin_ws

# get to your ROS workspace, in our case:
cd ~/catkin_ws/src

# install dependencies and clone repos to current working directory
wget -P /tmp https://raw.githubusercontent.com/UniBwTAS/continuous_clustering/master/scripts/install_dependencies_and_build.sh
bash /tmp/install_dependencies_and_build.sh
```
### Run Continuous Clustering

```bash
# run on kitti odometry dataset
roslaunch continuous_clustering demo_kitti_folder.launch path:=${SENSOR_DATA_PATH}/kitti_odometry/dataset/sequences

# run on VW Touareg rosbag
roslaunch continuous_clustering demo_touareg.launch bag_file:=${SENSOR_DATA_PATH}/rosbags/vw_touareg_example1.bag
```

## Evaluation on KITTI Odometry dataset

### With GUI (assumes prepared ROS setup)
```bash
# setup local installation or Docker with VNC (see above)

# generate labels (required only once)
# labels are saved to ${SENSOR_DATA_PATH}/kitti_odometry/dataset/sequences/<sequence>/labels_euclidean_clustering/
rosrun streaming_clustering gt_label_generator_tool ${SENSOR_DATA_PATH}/kitti_odometry/dataset/sequences

# run evaluation: for nice visualization argument 'delay_between_columns' specifies the artificial delay between two 
# range image columns in microseconds (below the delay is 1ms)
# scores for all sequences are printed at the end
roslaunch continuous_clustering demo_kitti_folder.launch path:=${SENSOR_DATA_PATH}/kitti_odometry/dataset/sequences delay_between_columns:=1000
```

### Minimal Docker Image (No ROS, No GUI)

```bash
# download kitti odometry dataset (see above)

# build docker image
cd /path/to/continous_clustering/folder # e.g. cd ~/catkin_ws/src/continuous_clustering
docker build -f build_no_ros.Dockerfile -t build_no_ros:v1 .

# generate labels (required only once)
# labels are saved to ${SENSOR_DATA_PATH}/kitti_odometry/dataset/sequences/<sequence>/labels_euclidean_clustering/
docker run --rm -v ${SENSOR_DATA_PATH}/kitti_odometry/dataset/sequences:/mnt/kitti_odometry/dataset/sequences --name build_no_ros build_no_ros:v1 gt_label_generator_tool /mnt/kitti_odometry/dataset/sequences

# run evaluation (scores for all sequences are printed at the end)
docker run --rm -v ${SENSOR_DATA_PATH}/kitti_odometry/dataset/sequences:/mnt/kitti_odometry/dataset/sequences --name build_no_ros build_no_ros:v1 kitti_demo /mnt/kitti_odometry/dataset/sequences
```

## Tipps for Rviz Visualization:

TODO

## Info about our LiDAR Drivers

## Velodyne

TODO

## Ouster

TODO

## TODOs
- Port it to ROS2: planned soon
- Add features:
  - Ground point segmentation:
    - In our paper we do ground point segmentation by using a accumulated terrain grid
    - However, the source code for our terrain estimation is not published yet
    - We do segmentation only based on current range image column (without temporal information)
    - This is slightly worse
    - TODO: publish source for terrain estimation
  - Publish [kitti2bag](https://github.com/tomas789/kitti2bag) fork, which splits the 360° PointCloud2 into individual firings in order to run it on KITTI rosbags