#include <ros/ros.h>

#include <continuous_clustering/clustering/continuous_clustering.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "continuous_clustering");

    continuous_clustering::ContinuousClustering node(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();

    return 0;
}
