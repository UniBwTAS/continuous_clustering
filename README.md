[![Basic Build Workflow](https://github.com/UniBwTAS/continuous_clustering/actions/workflows/basic-build-ci.yaml/badge.svg?branch=master)](https://github.com/UniBwTAS/continuous_clustering/actions/workflows/basic-build-ci.yaml)
[![Publish Docker image](https://github.com/UniBwTAS/continuous_clustering/actions/workflows/publish-docker-image.yaml/badge.svg)](https://github.com/UniBwTAS/continuous_clustering/actions/workflows/publish-docker-image.yaml)

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

#### Option 2: Locally on Ubuntu 20.04 (Focal) and ROS Noetic

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

### Results

The following results were obtained at Commit SHA [fa3c53b](https://github.com/UniBwTAS/continuous_clustering/commit/fa3c53bab51975b06ae5ec3a9e56567729149e4f)

#### Clustering

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

#### Ground Point Segmentation:

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
# build docker image (if not already done)
cd /path/to/continous_clustering/folder # e.g. cd ~/catkin_ws/src/continuous_clustering
docker build -f build_no_ros.Dockerfile -t build_no_ros:v1 .

# run evaluation (scores for all sequences are printed at the end)
docker run --rm -v ${KITTI_SEQUENCES_PATH}:/mnt/kitti_sequences --name build_no_ros build_no_ros:v1 kitti_demo /mnt/kitti_sequences

# if you want to cancel this container call from host
docker stop build_no_ros
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