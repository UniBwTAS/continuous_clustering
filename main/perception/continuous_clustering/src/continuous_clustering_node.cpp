#include <ros/ros.h>

#include <streaming_clustering/streaming_clustering.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "streaming_clustering");

    streaming_clustering::StreamingClustering node(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();

    return 0;
}
