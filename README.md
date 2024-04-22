# Low Latency Instance Segmentation by Continuous Clustering for Rotating LiDAR Sensors

[![Basic Build Workflow](https://github.com/UniBwTAS/continuous_clustering/actions/workflows/basic-build-ci.yaml/badge.svg?branch=master)](https://github.com/UniBwTAS/continuous_clustering/actions/workflows/basic-build-ci.yaml)
[![Publish Docker image](https://github.com/UniBwTAS/continuous_clustering/actions/workflows/publish-docker-image.yaml/badge.svg)](https://github.com/UniBwTAS/continuous_clustering/actions/workflows/publish-docker-image.yaml)
[![arXiv](https://img.shields.io/badge/arXiv-2311.13976-b31b1b.svg)](https://arxiv.org/abs/2311.13976)

![Continuous Clustering Demo](https://github.com/UniBwTAS/continuous_clustering/blob/master/assets/demo.gif)

## Abstract:

Low-latency instance segmentation of LiDAR point clouds is crucial in real-world applications because it serves as an
initial and frequently-used building block in a robot's perception pipeline, where every task adds further delay.
Particularly in dynamic environments, this total delay can result in significant positional offsets of dynamic objects,
as seen in highway scenarios. To address this issue, we employ continuous clustering of obstacle points in order to
obtain an instance-segmented point cloud. Unlike most existing approaches, which use a full revolution of the LiDAR
sensor, we process the data stream in a continuous and seamless fashion. More specifically, each column of a range image
is processed as soon it is available. Obstacle points are clustered to existing instances in real-time and it is checked
at a high-frequency which instances are completed and are ready to be published. An additional advantage is that no
problematic discontinuities between the points of the start and the end of a scan are observed. In this work we describe
the two-layered data structure and the corresponding algorithm for continuous clustering, which is able to cluster the
incoming data in real time. We explain the importance of a large perceptive field of view. Furthermore, we describe and
evaluate important architectural design choices, which could be relevant to design an architecture for deep learning
based low-latency instance segmentation.

## If you find our work useful in your research please consider citing our paper:

```
@misc{reich2023low,
      title={Low Latency Instance Segmentation by Continuous Clustering for Rotating LiDAR Sensors}, 
      author={Andreas Reich and Hans-Joachim Wuensche},
      year={2023},
      eprint={2311.13976},
      archivePrefix={arXiv},
      primaryClass={cs.CV}
}
```

Get PDF [here](https://arxiv.org/abs/2311.13976).

## Acknowledgement

The authors gratefully acknowledge funding by the Federal Office of Bundeswehr Equipment, Information Technology and
In-Service Support (BAAINBw).

## Overview

- [Examples](#examples)
- [Run it yourself](#run-it-yourself)
    - [1. Download Sensor Data](#1-download-sensor-data)
        - [1.1. SemanticKitti](#11-semantickitti)
        - [1.2. Rosbag of our test vehicle VW Touareg](#12-rosbag-of-our-test-vehicle-vw-touareg)
    - [2. Setup Environment](#2-setup-environment)
        - [2.1. Option: Docker + GUI (VNC)](#21-option-docker--gui-vnc)
        - [2.2. Option: Locally on Ubuntu 20.04 (Focal) and ROS Noetic](#22-option-locally-on-ubuntu-2004-focal-and-ros-noetic)
    - [3. Run Continuous Clustering](#3-run-continuous-clustering)
- [Evaluation on SemanticKITTI Dataset](#evaluation-on-semantickitti-dataset)
    - [1. Evaluation Results](#1-evaluation-results)
        - [1.1. Clustering](#11-clustering)
        - [1.2. Ground Point Segmentation](#12-ground-point-segmentation)
    - [2. Get Ground Truth Labels](#2-get-ground-truth-labels)
        - [2.1. Option: Download pre-generated labels](#21-option-download-pre-generated-labels-fastest)
        - [2.2. Option: Generate with GUI & ROS setup](#22-option-generate-with-gui--ros-setup-assumes-prepared-ros-setup-see-above-useful-for-debugging-etc)
        - [2.3. Option: Generate without GUI or ROS within Minimal Docker Container](#23-option-generate-without-gui-or-ros-within-minimal-docker-container)
    - [3. Run Evaluation](#3-run-evaluation)
        - [3.1. Option: Evaluate with GUI & ROS setup](#31-option-evaluate-with-gui--ros-setup-assumes-prepared-ros-setup-useful-for-debugging)
        - [3.2. Option: Evaluate without GUI or ROS within Minimal Docker Container](#32-option-evaluate-without-gui-or-ros-within-minimal-docker-container)
- [Info about our LiDAR Drivers](#info-about-our-lidar-drivers)
    - [1. Velodyne](#1-velodyne)
    - [2. Ouster](#2-ouster)
    - [3. Generic Points](#3-generic-points)
- [TODOs](#todos)

# Examples:

## Works with uncommon mounting positions

We mounted two Ouster OS 32 at a tilted angle in order to get rid of the blind spots of our main LiDAR sensor. Our
clustering also works with these mounting positions. The main challenge here is the ground point segmentation not the
clustering. It works ok, but we hope to improve it in the future.

![Clustering with Ouster sensor](https://github.com/UniBwTAS/continuous_clustering/blob/master/assets/demo_ouster.gif)

## Works with Fog

There are many clutter points and the camera image is almost useless. But the clustering still works quite well after
filtering potential fog points.

![Clustering with clutter points from fog](https://github.com/UniBwTAS/continuous_clustering/blob/master/assets/demo_fog.gif)

## Works on German Highway

There are often no speed limits on the German Highway. So it is not uncommon to see cars with velocities of 180 km/h or
much higher. A latency of e.g. 200ms leads to positional errors of `(180 / 3.6) m/s * 0.2s = 10m`. This shows the need
to keep latencies at a minimum.

[![Video GIF](https://user-images.githubusercontent.com/74038190/235294007-de441046-823e-4eff-89bf-d4df52858b65.gif)](https://www.youtube.com/watch?v=DZKuAQBngNE&t=98s)

# Run it yourself:

## 1. Download Sensor Data

### 1.1. SemanticKitti

We use the same folder structure as the SemanticKitti dataset:

![SemanticKitti Folder Structure](http://www.semantic-kitti.org/images/folder_structure.svg)

If it is already downloaded, then set the environment variable `KITTI_SEQUENCES_PATH` accordingly (i.e. parent folder of
folders `00`, `01`, `02`, ...):

```bash
export KITTI_SEQUENCES_PATH="/folder/containing/the/semantic/kitti/sequences"
```

If you don't have it, then you can download it automatically with our script:

```bash
# this takes quite some time -> skip this step if rosbag (below) is enough for you or you want to try it as fast as possible
cd /folder/with/enough/space
curl -s https://raw.githubusercontent.com/UniBwTAS/continuous_clustering/master/scripts/semantic_kitti_downloader.sh | bash -s
export KITTI_SEQUENCES_PATH="$(pwd)/kitti_odometry/dataset/sequences"
```

### 1.2. Rosbag of our test vehicle VW Touareg

Download the rosbag:

```bash
cd /folder/with/enough/space
pip3 install gdown && gdown 1zM4xPRahgxdJXJGHNXYUpM_g4-9UrcwC
export ROSBAG_PATH=$(pwd)
```

Alternatively download it manually from
our [Google Drive](https://drive.google.com/file/d/1zM4xPRahgxdJXJGHNXYUpM_g4-9UrcwC/view?usp=sharing) and set the
environment variable `ROSBAG_PATH` accordingly: `export ROSBAG_PATH=/parent/folder/of/rosbag`

Available bags:

- `gdown 1zM4xPRahgxdJXJGHNXYUpM_g4-9UrcwC` (
  3.9GB, [Manual Download](https://drive.google.com/file/d/1zM4xPRahgxdJXJGHNXYUpM_g4-9UrcwC/view?usp=sharing))
    - Long recording in urban scenario (no camera to reduce file size, no Ouster sensors)
- `gdown 1qjCG6-nWBZ_2wJwoP80jj0gGopBT2c23` (
  2.4GB, [Manual Download](https://drive.google.com/file/d/1qjCG6-nWBZ_2wJwoP80jj0gGopBT2c23/view?usp=sharing))
    - Recording including Ouster 32 sensor data (blurred camera for privacy reasons)
- `gdown 146IaBdEmkfBWdIgGV5HzrEYDTol84a1H` (
  0.7GB, [Manual Download](https://drive.google.com/file/d/146IaBdEmkfBWdIgGV5HzrEYDTol84a1H/view?usp=sharing))
    - Short recording of German Highway (blurred camera for privacy reasons)

## 2. Setup Environment

### 2.1. Option: Docker + GUI (VNC)

This option is the fastest to set up. However, due to missing hardware acceleration in the VNC Docker container for RVIZ
the rosbag is played at 1/10 speed.

1. Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/)
2. Ensure shell variables `KITTI_SEQUENCES_PATH` and/or `ROSBAG_PATH` are (still) set correctly (see above)
3. Pull and run docker container:

```bash
docker run -d -p 6080:80 -v /dev/shm:/dev/shm -v ${KITTI_SEQUENCES_PATH}:/mnt/kitti_sequences -v ${ROSBAG_PATH}:/mnt/rosbags -e KITTI_SEQUENCES_PATH=/mnt/kitti_sequences -e ROSBAG_PATH=/mnt/rosbags --name continuous_clustering_demo andreasr30/continuous_clustering_demo:master
```

4. Open your browser on host and enter: http://localhost:6080 (wait a few seconds and retry if it does not load)
5. Open terminal in browser window (click 'Start' -> 'System Tools' -> 'LXTerminal')
6. Continue with step "Run Continuous Clustering" (see below) in the terminal opened in step 2. (There you can use the
   clipboard feature of noVNC; tiny arrow on the left of the screen)

### 2.2. Option: Locally on Ubuntu 20.04 (Focal) and ROS Noetic

```bash
# install ROS (if not already installed)
wget -P /tmp https://raw.githubusercontent.com/UniBwTAS/continuous_clustering/master/scripts/install_ros.sh
bash /tmp/install_ros.sh

# setup ROS workspace (if not already existing)
wget -P /tmp https://raw.githubusercontent.com/UniBwTAS/continuous_clustering/master/scripts/setup_workspace.sh
bash /tmp/setup_workspace.sh # created at ~/catkin_ws

# switch to your ROS workspace, in our case:
cd ~/catkin_ws/src

# install dependencies and clone repos to current working directory
wget -P /tmp https://raw.githubusercontent.com/UniBwTAS/continuous_clustering/master/scripts/clone_repositories_and_install_dependencies.sh
bash /tmp/clone_repositories_and_install_dependencies.sh
catkin build
```

## 3. Run Continuous Clustering

```bash
# run on kitti odometry dataset
roslaunch continuous_clustering demo_kitti_folder.launch path:=${KITTI_SEQUENCES_PATH}

# run on VW Touareg rosbag (set the playback speed lower in Rviz (RosbagPanel) if you run in docker due to missing 
# hardware acceleration for graphical output)
roslaunch continuous_clustering demo_touareg.launch bag_file:=${ROSBAG_PATH}/vw_touareg_example1.bag
```

> [!Tip]
> For the latter launch file, you can use `--wait_for_tf:=false` (default: `true`) argument. It controls whether to wait
> for the transform from velodyne to fixed frame (e.g. odometry frame) with a timestamp larger than the one of the
> firing or whether to use the latest available (probably incorrect) transform. The former is the accurate approach
> (that's why it is the default) but the columns are published in larger batches/slices because they are accumulated
> between two transforms. The size of a slice depends on the update rate of the transform (i.e. transforms with 50Hz
> lead to batches/slices of 1/5 rotation for a LiDAR rotating with 10Hz). So for a nice visualization where the columns
> are published one by one like it the GIF at the top of the page you should disable this flag.

# Evaluation on SemanticKITTI Dataset

We evaluate our clustering algorithm with the same metrics as described in the paper _TRAVEL: Traversable Ground and
Above-Ground Object Segmentation Using Graph Representation of 3D LiDAR
Scans_ ([arXiv](https://arxiv.org/abs/2206.03190), [GitHub](https://github.com/url-kaist/TRAVEL)
, [IEEE Xplore](https://ieeexplore.ieee.org/document/9794594)), namely Over-Segmentation Entropy (OSE) /
Under-Segmentation Entropy (USE) for clustering performance and precision / recall / accuracy / F1-Score for ground
point segmentation.

## 1. Evaluation Results

> [!NOTE]
> The following results were obtained at Commit SHA [fa3c53b](https://github.com/UniBwTAS/continuous_clustering/commit/fa3c53bab51975b06ae5ec3a9e56567729149e4f)

### 1.1. Clustering

| Sequence | USE &mu; &darr; / &sigma; &darr; | OSE &mu; &darr; / &sigma; &darr; |
| :---: | :---: | :---: |
| All (**TRAVEL**)| 24.07 / 11.8 | 70.40 / 34.44 |
| All (**Ours**)| **18.75** / 7.82 | **40.22** / 14.56 |
| 0 | 15.30 / 4.57 | 39.80 / 10.84 |
| 1 | 17.88 / 9.66 | 35.81 / 13.92 |
| 2 | 16.13 / 5.63 | 27.67 / 11.54 |
| 3 | 16.30 / 7.12 | 33.08 / 18.37 |
| 4 | 17.50 / 13.80 | 39.61 / 17.83 |
| 5 | 22.44 / 4.82 | 49.58 / 8.82 |
| 6 | 36.90 / 5.35 | 57.27 / 8.35 |
| 7 | 17.17 / 5.02 | 40.78 / 12.58 |
| 8 | 19.11 / 7.12 | 48.58 / 12.47 |
| 9 | 18.45 / 6.25 | 39.62 / 11.86 |
| 10 | 20.10 / 8.70 | 34.33 / 12.37 |

### 1.2. Ground Point Segmentation

| Sequence | Recall &mu; &uarr; / &sigma; &darr; | Precision &mu; &uarr; / &sigma; &darr; | F1-Score &mu; &uarr; / &sigma; &darr; | Accuracy &mu; &uarr; / &sigma; &darr; |
| :---: | :---: | :---: | :---: | :---: |
| All (**TRAVEL**) | 90.0 / - | 96.7 / - | 93.1 / 4.3 | 93.9 / 3.7 |
| All (**Ours**) | 95.95 / 3.60 | 89.25 / 7.04 | 92.32 / 4.70 | 93.44 / 3.83 |
| 0 | 97.19 / 1.53 | 89.65 / 4.99 | 93.20 / 3.10 | 94.86 / 1.79 |
| 1 | 92.31 / 5.93 | 87.40 / 7.59 | 89.62 / 5.73 | 88.32 / 4.94 |
| 2 | 95.61 / 3.76 | 91.82 / 4.27 | 93.57 / 2.79 | 93.96 / 3.20 |
| 3 | 96.94 / 1.77 | 77.21 / 13.79 | 85.24 / 9.23 | 85.12 / 8.46 |
| 4 | 98.55 / 0.92 | 89.39 / 4.37 | 93.68 / 2.21 | 93.36 / 1.87 |
| 5 | 95.84 / 2.54 | 87.11 / 6.71 | 91.18 / 4.67 | 93.33 / 2.30 |
| 6 | 97.71 / 1.28 | 95.83 / 1.50 | 96.75 / 1.13 | 95.87 / 1.19 |
| 7 | 96.97 / 1.93 | 90.55 / 4.31 | 93.60 / 2.73 | 95.20 / 1.35 |
| 8 | 96.44 / 2.43 | 89.46 / 7.64 | 92.66 / 4.85 | 94.38 / 2.56 |
| 9 | 95.31 / 4.03 | 88.22 / 5.70 | 91.45 / 3.37 | 91.74 / 3.20 |
| 10 | 91.62 / 6.79 | 85.76 / 7.22 | 88.33 / 5.45 | 91.83 / 3.63 |

## 2. Get Ground Truth Labels

In order to evaluate OSE and USE for clustering performance additional labels are required, which are generated from the
Semantic Kitti Labels and using a euclidean distance-based clustering.
See [Issue](https://github.com/url-kaist/TRAVEL/issues/6) in TRAVEL GitHub repository
and [src/evaluation/kitti_evaluation.cpp](src/evaluation/kitti_evaluation.cpp) for more details.

### 2.1. Option: Download pre-generated labels (fastest)

```bash
cd /tmp
pip3 install gdown && gdown 1MOfLbUQcwRMLhRca0bxJMLVriU3G8Tg3
cd ${KITTI_SEQUENCES_PATH}/../..
unzip /tmp/data_odometry_labels_euclidean_clustering.zip
```

Alternatively download it manually from
our [Google Drive](https://drive.google.com/file/d/1MOfLbUQcwRMLhRca0bxJMLVriU3G8Tg3/view?usp=sharing) and unzip it to
the correct location (in parent directory of `dataset` folder).

### 2.2. Option: Generate with GUI & ROS setup (assumes prepared ROS setup, see above, useful for debugging etc.)

Generate labels, which are saved to `${KITTI_SEQUENCES_PATH}/<sequence>/labels_euclidean_clustering/`

```bash
rosrun continuous_clustering gt_label_generator_tool ${KITTI_SEQUENCES_PATH} --no-ros --num-threads 8
```

> [!TIP]
> If you want to visualize the generated ground truth labels in an online fashion then remove the `--no-ros` flag and
> use just one thread (default). You can then subscribe to the corresponding topic in Rviz and visualize the point
> labels.

### 2.3. Option: Generate without GUI or ROS within Minimal Docker Container

```bash
# build docker image
cd /path/to/continous_clustering/folder # e.g. cd ~/catkin_ws/src/continuous_clustering
docker build -f build_no_ros.Dockerfile -t build_no_ros:v1 .

# generate labels (required only once)
# ground truth labels are saved to ${KITTI_SEQUENCES_PATH}/<sequence>/labels_euclidean_clustering/
docker run --rm -v ${KITTI_SEQUENCES_PATH}:/mnt/kitti_sequences --name build_no_ros build_no_ros:v1 gt_label_generator_tool /mnt/kitti_sequences --num-threads 8

# if you want to cancel this container call from host
docker stop build_no_ros
```

## 3. Run Evaluation

### 3.1. Option: Evaluate with GUI & ROS setup (assumes prepared ROS setup; useful for debugging)

```bash
# run evaluation slowly with visual output
# use --delay-between-columns to adjust artificial delay in microseconds between two range image columns (default: 2000)
roslaunch continuous_clustering demo_kitti_folder.launch path:=${KITTI_SEQUENCES_PATH} evaluate:=true

# alternatively: run evaluation as fast as possible (scores for all sequences are printed at the end)
roslaunch continuous_clustering demo_kitti_folder.launch path:=${KITTI_SEQUENCES_PATH} evaluate-fast:=true
```

### 3.2. Option: Evaluate without GUI or ROS within Minimal Docker Container

```bash
# build docker image (if not already done)
cd /path/to/continous_clustering/folder # e.g. cd ~/catkin_ws/src/continuous_clustering
docker build -f build_no_ros.Dockerfile -t build_no_ros:v1 .

# run evaluation (scores for all sequences are printed at the end)
docker run --rm -v ${KITTI_SEQUENCES_PATH}:/mnt/kitti_sequences --name build_no_ros build_no_ros:v1 kitti_demo /mnt/kitti_sequences

# if you want to cancel this container call from host
docker stop build_no_ros
```

# Info about our LiDAR Drivers

Our clustering algorithm is able to process the UDP packets from the LiDAR sensor. So the firings can be processed
immediately. We directly process the raw UDP packets from the corresponding sensor, which makes the input
manufacturer/sensor specific. Luckily, we can use external libraries, so it is not necessary reimplement the decoding
part (UDP Packet -> Euclidean Point Positions). Currently, we support following sensor manufacturers:

## 1. Velodyne

- Tested Sensors: VLS 128
- All other rotating Velodyne LiDARs should work, too
- We use this project: [ros-drivers/velodyne](https://github.com/ros-drivers/velodyne)
- Unfortunately, this projects always records full rotations (ROS message type `velodyne_msgs/VelodyneScan`)
    - Therefore, we use this project [UniBwTAS/ethernet_bridge](https://github.com/UniBwTAS/ethernet_bridge) to read and
      record the raw UDP packets from the sensor (ROS message type: `ethernet_msgs/Packet`)
    - Example launch file to start ethernet_bridge on real robot (e.g.: PORT 2370):

```xml

<launch>
    <!-- flag if in real robot or playing a rosbag -->
    <arg name="rosbag" default="false"/>

    <arg name="port" default="2370"/>
    <arg name="ns_ethernet_bridge" value="bus/vls128_roof/eth_scan"/>

    <node pkg="ethernet_bridge" type="udp" name="ethernet_bridge" unless="$(arg rosbag)" ns="$(arg ns_ethernet_bridge)">
        <param name="ethernet_bindAddress" value="0.0.0.0"/>
        <param name="ethernet_bindPort" value="$(arg port)"/>
    </node>
</launch>
```

- Our clustering subscribes to these UDP packets and can use the middleware-agnostic library
  from [ros-drivers/velodyne](https://github.com/ros-drivers/velodyne) to decode packets to euclidean points
    - See source code at: [velodyne_input.hpp](include/continuous_clustering/ros/velodyne_driver.hpp)

## 2. Ouster

- Tested Sensors: OS 32
- All other rotating Ouster LiDARs should work, too
- We use this project: [ouster-lidar/ouster-ros](https://github.com/ouster-lidar/ouster-ros)
- This project reads and records individual UDP packets
- Therefore, the sensors can be launched regularly with the launch files
  from [ouster-lidar/ouster-ros](https://github.com/ouster-lidar/ouster-ros)
- Our clustering subscribes to Ouster's UDP packets (ROS Message type: `ouster_ros/PacketMsg`) and can use the library
  from [ouster-lidar/ouster-ros](https://github.com/ouster-lidar/ouster-ros) to decode packets to euclidean points
    - See source code at: [ouster_input.hpp](include/continuous_clustering/ros/ouster_driver.hpp)

## 3. Generic Points

Alternatively, you can also launch the clustering with parameter `sensor_manufacturer:="generic_points"`. Then it
subscribes to `<namespace>/raw_data` topic and expects `sensor_msgs::PointCloud2` messages, which should represent one
firing. It has to be an organized point cloud with `width=1` and `height=<number-of-lasers>`. Required point fields are
`x`, `y`, `z` (w.r.t. sensor origin), and `intensity`. Missing returns should be represented by `NaN`.
See [launch/sensor_kitti.launch](launch/sensor_kitti.launch) for such an example.

# TODOs

- Port it to ROS2: planned soon
- Add features:
    - Ground point segmentation:
        - In our paper we do ground point segmentation by using an accumulated terrain grid
        - However, the source code for our terrain estimation is not published yet
        - We do segmentation only based on current range image column (without temporal information)
        - This is slightly worse
        - TODO: publish source for terrain estimation
    - Publish [kitti2bag](https://github.com/tomas789/kitti2bag) fork, which splits the 360° PointCloud2 into individual
      firings in order to run it on KITTI rosbags