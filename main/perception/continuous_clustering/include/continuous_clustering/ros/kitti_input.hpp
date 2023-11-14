#ifndef CONTINUOUS_CLUSTERING_KITTI_INPUT_HPP
#define CONTINUOUS_CLUSTERING_KITTI_INPUT_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <continuous_clustering/ros/ros_sensor_input.hpp>

namespace continuous_clustering
{

class KittiInput : public RosSensorInput<sensor_msgs::PointCloud2>
{
  public:
    KittiInput(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : RosSensorInput(nh, nh_private)
    {
    }

  protected:
    void onRawDataArrived(const sensor_msgs::PointCloud2::ConstPtr& m) override
    {
        if (m->header.stamp.isZero())
            return;

        if (firing_index == 0)
        {
            num_lasers = static_cast<int>(m->height);
            prepareNewFiring();
        }

        sensor_msgs::PointCloud2ConstIterator<float> iter_x_in(*m, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y_in(*m, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z_in(*m, "z");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity_in(*m, "intensity");
        sensor_msgs::PointCloud2ConstIterator<double> iter_kitti_index_in(*m, "original_kitti_index");

        int row_index = 0;
        uint64_t stamp = m->header.stamp.toNSec();
        for (; iter_x_in != iter_x_in.end();
             ++iter_x_in, ++iter_y_in, ++iter_z_in, ++iter_intensity_in, ++iter_kitti_index_in, ++row_index)
        {
            current_firing->points[row_index].stamp = stamp;
            current_firing->points[row_index].firing_index = firing_index;
            current_firing->points[row_index].x = *iter_x_in;
            current_firing->points[row_index].y = *iter_y_in;
            current_firing->points[row_index].z = *iter_z_in;
            current_firing->points[row_index].intensity = static_cast<uint8_t>(*iter_intensity_in * 255);
            current_firing->points[row_index].globally_unique_point_index =
                *(reinterpret_cast<const uint64_t*>(&(*iter_kitti_index_in))); // PointCloud2 does not support UINT64
        }

        keepTrackOfMinAndMaxStamp(stamp); // same for firing -> enough to call once

        publishCurrentFiringAndPrepareNewFiring();
    }
};

} // namespace continuous_clustering
#endif
