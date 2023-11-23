FROM ubuntu:focal

ENV HOME /root
ENV DEBIAN_FRONTEND noninteractive

# install build tools
RUN apt update && apt install -y -q \
    git \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# install minimal dependencies
# unfortunately the point cloud library (PCL) is very large (we install only minimal necessary libraries)
RUN apt update && apt install -y -q \
    libeigen3-dev \
    libboost-all-dev \
    libflann-dev \
    libpcl-common* \
    libpcl-segmentation* \
    && rm -rf /var/lib/apt/lists/*

# install PCL itself (mainly for headers & cmake) without installing all its dependencies (only possible with dpkg)
WORKDIR /tmp
RUN apt update \
    && apt download libpcl-dev \
    && dpkg -i --force-depends libpcl-dev*.deb \
    && rm -rf /var/lib/apt/lists/*

# install dependencies and build workspace
WORKDIR $HOME
COPY . ./continuous_clustering
RUN cd continuous_clustering \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make \
    && cp gt_label_generator_tool /bin \
    && cp kitti_demo /bin \
    && cd .. \
    && rm -rf build

# run it
CMD ["kitti_demo", "/mnt/kitti_odometry/dataset/sequences/"]

