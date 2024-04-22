#ifndef CONTINUOUS_CLUSTERING_GENERIC_POINTS_DRIVER_HPP
#define CONTINUOUS_CLUSTERING_GENERIC_POINTS_DRIVER_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <continuous_clustering/clustering/pipeline_nodes/driver.hpp>

namespace continuous_clustering
{

class GenericPointsDriver : public Driver<sensor_msgs::PointCloud2::ConstPtr>
{
  public:
    explicit GenericPointsDriver(const ros::NodeHandle& nh_private, uint8_t num_threads = 1) : Driver(num_threads)
    {
    }

  protected:
    void processJob(sensor_msgs::PointCloud2::ConstPtr&& job) override
    {
        if (job->header.stamp.isZero())
            return;

        if (firing_index == 0)
        {
            num_lasers = static_cast<int>(job->height);
            prepareNewFiring();
        }

        sensor_msgs::PointCloud2ConstIterator<float> iter_x_in(*job, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y_in(*job, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z_in(*job, "z");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity_in(*job, "intensity");

        int row_index = 0;
        int64_t stamp = job->header.stamp.toNSec();
        for (; iter_x_in != iter_x_in.end(); ++iter_x_in, ++iter_y_in, ++iter_z_in, ++iter_intensity_in, ++row_index)
        {
            current_firing->points[row_index].stamp = stamp;
            current_firing->points[row_index].firing_index = firing_index;
            current_firing->points[row_index].x = *iter_x_in;
            current_firing->points[row_index].y = *iter_y_in;
            current_firing->points[row_index].z = *iter_z_in;
            current_firing->points[row_index].intensity = static_cast<uint8_t>(*iter_intensity_in * 255);
            current_firing->points[row_index].globally_unique_point_index = 0; // only required for evaluation
        }

        keepTrackOfMinAndMaxStamp(stamp); // same for firing -> enough to call once

        publishCurrentFiringAndPrepareNewFiring();
    }
};

} // namespace continuous_clustering
#endif
