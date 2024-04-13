cmake_minimum_required(VERSION 3.16...3.26)
project(continuous_clustering VERSION 0.0.1 LANGUAGES CXX)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
add_compile_options(-pthread)

# recommended when using perf profiler
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common segmentation)

if ("$ENV{ROS_VERSION}" STREQUAL "1")
    message(STATUS "steaming_clustering ROS 1 wrapper will be compiled")
    add_compile_definitions(WITH_ROS)
    set(COMPILE_WITH_ROS true)

    # ROS/catkin specific setup
    find_package(catkin REQUIRED roscpp grid_map_msgs sensor_msgs dynamic_reconfigure velodyne_pointcloud ouster_ros pcl_conversions ethernet_msgs)
    generate_dynamic_reconfigure_options(cfg/ContinuousClustering.cfg)
    catkin_package(
            INCLUDE_DIRS include
            LIBRARIES kitti_loader kitti_evaluation continuous_clustering_ros
            CATKIN_DEPENDS roscpp sensor_msgs grid_map_msgs dynamic_reconfigure velodyne_pointcloud ouster_ros
            DEPENDS PCL EIGEN3
    )
endif ()

include_directories(include ${EIGEN3_INCLUDE_DIRS})

# library to load kitti odometry dataset
add_library(kitti_loader src/evaluation/kitti_loader.cpp)

# library to evaluate ground point segmentation (recall, ...) and clustering (over-/under-segmentation entropy)
add_library(kitti_evaluation src/evaluation/kitti_evaluation.cpp)
target_link_libraries(kitti_evaluation kitti_loader pcl_common pcl_segmentation)

# a tool which generates the ground truth labels for clustering evaluation
add_executable(gt_label_generator_tool src/tools/gt_label_generator_tool.cpp)
target_link_libraries(gt_label_generator_tool
        kitti_loader
        kitti_evaluation
        pthread
        )

# a tool which is able to run clustering directly on kitti dataset (raw files) instead of rosbag (and runs evaluation)
# add_executable(kitti_demo src/tools/kitti_demo.cpp)
# target_link_libraries(kitti_demo
#         continuous_clustering
#         kitti_loader
#         kitti_evaluation
#         )

if (${COMPILE_WITH_ROS})
    include_directories(${catkin_INCLUDE_DIRS})

    # helper functions to publish clustering result to ROS
    add_library(continuous_clustering_ros src/ros/ros_utils.cpp)
    target_link_libraries(continuous_clustering_ros ${catkin_LIBRARIES})

    # a ROS node (wrapper), which subscribes to raw sensor data and publishes the clustering result
    add_executable(continuous_clustering_node src/ros/continuous_clustering_node.cpp)
    target_link_libraries(continuous_clustering_node
            ${catkin_LIBRARIES}
            continuous_clustering_ros
            )
    add_dependencies(continuous_clustering_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    # link tools against ROS-specific libraries
    target_link_libraries(gt_label_generator_tool ${catkin_LIBRARIES})
    # target_link_libraries(kitti_demo ${catkin_LIBRARIES} continuous_clustering_ros)
endif ()
