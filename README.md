[![Basic Build Workflow](https://github.com/UniBwTAS/continuous_clustering/actions/workflows/basic-build-ci.yaml/badge.svg?branch=master)](https://github.com/UniBwTAS/continuous_clustering/actions/workflows/basic-build-ci.yaml)

## Low Latency Instance Segmentation by Continuous Clustering for Rotating LiDAR Sensors

![](https://github.com/UniBwTAS/continuous_clustering/blob/master/assets/demo.gif)

### Abstract:

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

### If you find our work useful in your research please consider citing our paper:

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

## Run it yourself:

### Download Sensor Data

#### SemanticKitti

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

#### Rosbag of our test vehicle VW Touareg

Download the rosbag:

```bash
cd /folder/with/enough/space
pip3 install gdown && gdown 1zM4xPRahgxdJXJGHNXYUpM_g4-9UrcwC
export ROSBAG_PATH=$(pwd)
```

Alternatively download it manually from
our [Google Drive](https://drive.google.com/file/d/1zM4xPRahgxdJXJGHNXYUpM_g4-9UrcwC/view?usp=sharing) and set the
environment variable `ROSBAG_PATH` to download folder:

```bash
export ROSBAG_PATH=/download/folder/of/rosbag/file
```

### Setup Environment

#### Option 1: Docker + GUI (VNC):

This option is the fastest to set up. However, due to missing hardware acceleration in the VNC Docker container for RVIZ
the rosbag is played at 1/10 speed.

```bash
# IMPORTANT: this option is 

# install docker engine (e.g.: https://docs.docker.com/engine/install/ubuntu/)

# ensure shell variables KITTI_SEQUENCES_PATH and/or ROSBAG_PATH are (still) set correctly (see above)

# pull and run docker container
docker run -d -p 6080:80 -v /dev/shm:/dev/shm -v ${KITTI_SEQUENCES_PATH}:/mnt/kitti_sequences -v ${ROSBAG_PATH}:/mnt/rosbags -e KITTI_SEQUENCES_PATH=/mnt/kitti_sequences -e ROSBAG_PATH=/mnt/rosbags --name continuous_clustering_demo andreasr30/continuous_clustering_demo:focal-ros1-vnc

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
wget -P /tmp https://raw.githubusercontent.com/UniBwTAS/continuous_clustering/master/scripts/clone_repositories_and_install_dependencies.sh
bash /tmp/clone_repositories_and_install_dependencies.sh
catkin build
```

### Run Continuous Clustering

```bash
# run on kitti odometry dataset
roslaunch continuous_clustering demo_kitti_folder.launch path:=${KITTI_SEQUENCES_PATH}

# run on VW Touareg rosbag
roslaunch continuous_clustering demo_touareg.launch bag_file:=${ROSBAG_PATH}/vw_touareg_example1.bag
```

## Evaluation on SemanticKITTI Dataset

We evaluate our clustering algorithm with the same metrics as described in the paper _TRAVEL: Traversable Ground and
Above-Ground Object Segmentation Using Graph Representation of 3D LiDAR
Scans_ ([arXiv](https://arxiv.org/abs/2206.03190), [GitHub](https://github.com/url-kaist/TRAVEL)
, [IEEE Xplore](https://ieeexplore.ieee.org/document/9794594)), namely Over-Segmentation Entropy (OSE) /
Under-Segmentation Entropy (USE) for clustering performance and precision / recall / accuracy / F1-Score for ground
point segmentation.

### Download/Generate Ground Truth Data

In order to evaluate OSE and USE for clustering performance additional labels are required, which are generated from the
Semantic Kitti Labels and using a euclidean distance-based clustering.
See [Issue](https://github.com/url-kaist/TRAVEL/issues/6) in TRAVEL GitHub repository
and [src/evaluation/kitti_evaluation.cpp](src/evaluation/kitti_evaluation.cpp) for more details.

#### Option 1: Download pre-generated labels

```bash
cd /tmp
pip3 install gdown && gdown 1lA-OHRg6yM8z7INF_7_03hB0yJ4RswaV
cd ${KITTI_SEQUENCES_PATH}/../..
unzip /tmp/data_odometry_labels_euclidean_clustering.zip
```

Alternatively download it manually from
our [Google Drive](https://drive.google.com/file/d/1lA-OHRg6yM8z7INF_7_03hB0yJ4RswaV/view?usp=sharing) and unzip it to
the correct location (in parent directory of `dataset` folder).

#### Option 2: Generate with GUI & ROS setup (assumes prepared ROS setup, see above, useful for debugging etc.)

Generate labels, which are saved to `${KITTI_SEQUENCES_PATH}/<sequence>/labels_euclidean_clustering/`
If you want to visualize the generated ground truth labels in ROS then remove the `--no-ros` flag and use just one
thread (default).

```bash
rosrun continuous_clustering gt_label_generator_tool ${KITTI_SEQUENCES_PATH} --no-ros --num-threads 8
```

#### Option 3: Generate without GUI or ROS within Minimal Docker Container

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

### Run Evaluation

#### Option 1: Evaluate with GUI & ROS setup (assumes prepared ROS setup, see above, useful for debugging)

```bash
# run evaluation slowly with visual output
# use --delay-between-columns to adjust artificial delay in microseconds between two range image columns (default: 2000)
roslaunch continuous_clustering demo_kitti_folder.launch path:=${KITTI_SEQUENCES_PATH} evaluate:=true

# alternatively: run evaluation as fast as possible (scores for all sequences are printed at the end)
roslaunch continuous_clustering demo_kitti_folder.launch path:=${KITTI_SEQUENCES_PATH} evaluate-fast:=true
```

#### Option 2: Evaluate without GUI or ROS within Minimal Docker Container

```bash
# run evaluation (scores for all sequences are printed at the end)
docker run --rm -v ${KITTI_SEQUENCES_PATH}:/mnt/kitti_sequences --name build_no_ros build_no_ros:v1 kitti_demo /mnt/kitti_sequences
```

## Tips for Rviz Visualization:

TODO

## Info about our LiDAR Drivers

### Velodyne

TODO

### Ouster

TODO

## TODOs

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