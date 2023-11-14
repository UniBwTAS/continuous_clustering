#ifndef CONTINUOUS_CLUSTERING_ROS_UTILS_HPP
#define CONTINUOUS_CLUSTERING_ROS_UTILS_HPP

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_broadcaster.h>

#include <continuous_clustering/clustering/continuous_clustering.hpp>
#include <continuous_clustering/clustering/point_types.hpp>
#include <continuous_clustering/evaluation/kitti_evaluation.hpp>

namespace continuous_clustering
{

struct PointCloud2Iterators
{
    // range image generation
    sensor_msgs::PointCloud2Iterator<float> iter_x_out;
    sensor_msgs::PointCloud2Iterator<float> iter_y_out;
    sensor_msgs::PointCloud2Iterator<float> iter_z_out;
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_f_out;
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_i_out;
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_time_sec_out;
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_time_nsec_out;
    sensor_msgs::PointCloud2Iterator<float> iter_d_out;
    sensor_msgs::PointCloud2Iterator<float> iter_a_out;
    sensor_msgs::PointCloud2Iterator<float> iter_ia_out;
    sensor_msgs::PointCloud2Iterator<double> iter_ca_out;
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_gc_out;
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_lc_out;
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_r_out;
    sensor_msgs::PointCloud2Iterator<double> iter_gpi_out;

    // ground point segmentation
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_gp_label_out;
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_dbg_gp_label_out;
    sensor_msgs::PointCloud2Iterator<int32_t> iter_dbg_c_n_left_out;
    sensor_msgs::PointCloud2Iterator<int32_t> iter_dbg_c_n_right_out;
    sensor_msgs::PointCloud2Iterator<float> iter_height_over_ground_out;

    // continuous clustering
    sensor_msgs::PointCloud2Iterator<double> iter_finished_at_azimuth_angle;
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_num_child_points;
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_tree_root_row_index;
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_tree_root_column_index;
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_number_of_visited_neighbors;
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_tree_id;
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_id;
};

sensor_msgs::PointCloud2Ptr clusterToPointCloud(const std::vector<Point>& cluster_points,
                                                int num_rows_in_range_image,
                                                uint64_t stamp_cluster,
                                                const std::string& frame_id);

sensor_msgs::PointCloud2Ptr columnToPointCloud(const ContinuousClustering& clustering,
                                               int64_t from_global_column_index,
                                               int64_t to_global_column_index,
                                               const std::string& frame_id);

sensor_msgs::PointCloud2Ptr firingToPointCloud(const RawPoints::ConstPtr& firing, const std::string& frame_id);

PointCloud2Iterators prepareMessageAndCreateIterators(sensor_msgs::PointCloud2& msg);
void addPointToMessage(PointCloud2Iterators& container, int data_index_message, const Point& point, int num_rows);
void addRawPointToMessage(PointCloud2Iterators& container, int data_index_message, const RawPoint& point);

sensor_msgs::PointCloud2Ptr
evaluationToPointCloud(const std::vector<KittiSegmentationEvaluationPoint>& point_cloud);

void publish_tf(tf2_ros::TransformBroadcaster& pub, const Eigen::Isometry3d& odom_from_velodyne, uint64_t stamp);
void publish_clock(ros::Publisher& pub, uint64_t stamp);
void publish_ego_robot_bounding_box(uint64_t stamp,
                                    ros::Publisher& pub,
                                    const ContinuousGroundSegmentationConfiguration& config);
} // namespace continuous_clustering

#endif