#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/Marker.h>

#include <continuous_clustering/ros/ros_utils.hpp>

namespace continuous_clustering
{

sensor_msgs::PointCloud2Ptr clusterToPointCloud(const std::vector<Point>& cluster_points,
                                                int num_rows_in_range_image,
                                                uint64_t stamp_cluster,
                                                const std::string& frame_id)
{
    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header.stamp.fromNSec(stamp_cluster);
    msg->header.frame_id = frame_id;
    msg->width = cluster_points.size();
    msg->height = 1;

    PointCloud2Iterators container = prepareMessageAndCreateIterators(*msg);

    int data_index_message = 0;
    for (const Point& point : cluster_points)
    {
        addPointToMessage(container, data_index_message, point, num_rows_in_range_image);
        data_index_message++;
    }

    return msg;
}

sensor_msgs::PointCloud2Ptr columnToPointCloud(const ContinuousClustering& clustering,
                                               int64_t from_global_column_index,
                                               int64_t to_global_column_index,
                                               const std::string& frame_id)
{
    int num_columns_to_publish = static_cast<int>(to_global_column_index - from_global_column_index) + 1;
    if (num_columns_to_publish <= 0)
        return nullptr;

    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header.frame_id = frame_id;
    msg->width = num_columns_to_publish;
    msg->height = clustering.num_rows_;

    PointCloud2Iterators container = prepareMessageAndCreateIterators(*msg);

    // keep track of minimum point stamp in this message
    uint64_t minimum_point_stamp = std::numeric_limits<uint64_t>::max();

    for (int message_column_index = 0; message_column_index < msg->width; ++message_column_index)
    {
        int ring_buffer_local_column_index =
            static_cast<int>((from_global_column_index + message_column_index) % clustering.ring_buffer_max_columns);
        for (int row_index = 0; row_index < clustering.num_rows_; ++row_index)
        {
            const Point& point =
                clustering.range_image_[ring_buffer_local_column_index * clustering.num_rows_ + row_index];
            int data_index_message = row_index * static_cast<int>(msg->width) + message_column_index;
            addPointToMessage(container, data_index_message, point, clustering.num_rows_);

            if (point.stamp != 0 && point.stamp < minimum_point_stamp)
                minimum_point_stamp = point.stamp;
        }
    }

    if (minimum_point_stamp != std::numeric_limits<uint64_t>::max())
        msg->header.stamp.fromNSec(minimum_point_stamp);
    else
        ROS_WARN_STREAM("This column had no timestamps. Unable to publish message with timestamp. Local Column Index: "
                        << from_global_column_index % clustering.num_columns_ << ", " << num_columns_to_publish);

    return msg;
}

sensor_msgs::PointCloud2Ptr firingToPointCloud(const RawPoints::ConstPtr& firing, const std::string& frame_id)
{
    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header.frame_id = frame_id;
    msg->width = 1;
    msg->height = firing->points.size();

    PointCloud2Iterators container = prepareMessageAndCreateIterators(*msg);

    // keep track of minimum point stamp in this message
    uint64_t minimum_point_stamp = std::numeric_limits<uint64_t>::max();

    int data_index_message = 0;
    for (const RawPoint& point : firing->points)
    {
        addRawPointToMessage(container, data_index_message, point);

        if (point.stamp != 0 && point.stamp < minimum_point_stamp)
            minimum_point_stamp = point.stamp;

        data_index_message++;
    }

    if (minimum_point_stamp != std::numeric_limits<uint64_t>::max())
        msg->header.stamp = ros::Time().fromNSec(minimum_point_stamp);

    return msg;
}

PointCloud2Iterators prepareMessageAndCreateIterators(sensor_msgs::PointCloud2& msg)
{
    msg.is_bigendian = false;
    msg.is_dense = false;

    sensor_msgs::PointCloud2Modifier output_modifier(msg);
    output_modifier.setPointCloud2Fields(27,
                                         "x",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "y",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "z",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "firing_index",
                                         1,
                                         sensor_msgs::PointField::UINT32,
                                         "intensity",
                                         1,
                                         sensor_msgs::PointField::UINT8,
                                         "time_sec",
                                         1,
                                         sensor_msgs::PointField::UINT32,
                                         "time_nsec",
                                         1,
                                         sensor_msgs::PointField::UINT32,
                                         "distance",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "azimuth_angle",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "inclination_angle",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "continuous_azimuth_angle",
                                         1,
                                         sensor_msgs::PointField::FLOAT64,
                                         "global_column_index",
                                         1,
                                         sensor_msgs::PointField::UINT32,
                                         "local_column_index",
                                         1,
                                         sensor_msgs::PointField::UINT16,
                                         "row_index",
                                         1,
                                         sensor_msgs::PointField::UINT16,
                                         "globally_unique_point_index",
                                         1,
                                         sensor_msgs::PointField::FLOAT64,
                                         "ground_point_label",
                                         1,
                                         sensor_msgs::PointField::UINT8,
                                         "debug_ground_point_label",
                                         1,
                                         sensor_msgs::PointField::UINT8,
                                         "debug_local_column_index_of_left_ground_neighbor",
                                         1,
                                         sensor_msgs::PointField::INT32,
                                         "debug_local_column_index_of_right_ground_neighbor",
                                         1,
                                         sensor_msgs::PointField::INT32,
                                         "height_over_ground",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "finished_at_continuous_azimuth_angle",
                                         1,
                                         sensor_msgs::PointField::FLOAT64,
                                         "num_child_points",
                                         1,
                                         sensor_msgs::PointField::UINT16,
                                         "tree_root_row_index",
                                         1,
                                         sensor_msgs::PointField::UINT16,
                                         "tree_root_column_index",
                                         1,
                                         sensor_msgs::PointField::UINT8,
                                         "number_of_visited_neighbors",
                                         1,
                                         sensor_msgs::PointField::UINT32,
                                         "tree_id",
                                         1,
                                         sensor_msgs::PointField::UINT32,
                                         "id",
                                         1,
                                         sensor_msgs::PointField::UINT32);

    return {{msg, "x"},
            {msg, "y"},
            {msg, "z"},
            {msg, "firing_index"},
            {msg, "intensity"},
            {msg, "time_sec"},
            {msg, "time_nsec"},
            {msg, "distance"},
            {msg, "azimuth_angle"},
            {msg, "inclination_angle"},
            {msg, "continuous_azimuth_angle"},
            {msg, "global_column_index"},
            {msg, "local_column_index"},
            {msg, "row_index"},
            {msg, "globally_unique_point_index"},
            {msg, "ground_point_label"},
            {msg, "debug_ground_point_label"},
            {msg, "debug_local_column_index_of_left_ground_neighbor"},
            {msg, "debug_local_column_index_of_right_ground_neighbor"},
            {msg, "height_over_ground"},
            {msg, "finished_at_continuous_azimuth_angle"},
            {msg, "num_child_points"},
            {msg, "tree_root_row_index"},
            {msg, "tree_root_column_index"},
            {msg, "number_of_visited_neighbors"},
            {msg, "tree_id"},
            {msg, "id"}};
}

void addPointToMessage(PointCloud2Iterators& container, int data_index_message, const Point& point, int num_rows)
{
    ros::Time stamp;
    stamp.fromNSec(point.stamp); // TODO: avoid this?

    *(container.iter_x_out + data_index_message) = point.xyz.x;
    *(container.iter_y_out + data_index_message) = point.xyz.y;
    *(container.iter_z_out + data_index_message) = point.xyz.z;
    *(container.iter_f_out + data_index_message) = static_cast<uint32_t>(point.firing_index);
    *(container.iter_i_out + data_index_message) = point.intensity;
    *(container.iter_time_sec_out + data_index_message) = stamp.sec;
    *(container.iter_time_nsec_out + data_index_message) = stamp.nsec;
    *(container.iter_d_out + data_index_message) = point.distance;
    *(container.iter_a_out + data_index_message) = point.azimuth_angle;
    *(container.iter_ia_out + data_index_message) = point.inclination_angle;
    *(container.iter_ca_out + data_index_message) = point.continuous_azimuth_angle;
    *(container.iter_gc_out + data_index_message) = point.global_column_index;
    *(container.iter_lc_out + data_index_message) = point.local_column_index;
    *(container.iter_r_out + data_index_message) = point.row_index;
    *(container.iter_gpi_out + data_index_message) =
        *reinterpret_cast<const double*>(&point.globally_unique_point_index); // PointCloud2 does not support UINT64
    *(container.iter_gp_label_out + data_index_message) = point.ground_point_label;
    *(container.iter_dbg_gp_label_out + data_index_message) = point.debug_ground_point_label;
    *(container.iter_dbg_c_n_left_out + data_index_message) = point.local_column_index_of_left_ground_neighbor;
    *(container.iter_dbg_c_n_right_out + data_index_message) = point.local_column_index_of_right_ground_neighbor;
    *(container.iter_height_over_ground_out + data_index_message) = point.height_over_ground;
    *(container.iter_finished_at_azimuth_angle + data_index_message) = point.finished_at_continuous_azimuth_angle;
    *(container.iter_num_child_points + data_index_message) = point.child_points.size();
    *(container.iter_tree_root_row_index + data_index_message) = point.tree_root_.row_index;
    *(container.iter_tree_root_column_index + data_index_message) = point.tree_root_.column_index;
    *(container.iter_number_of_visited_neighbors + data_index_message) = point.number_of_visited_neighbors;
    *(container.iter_tree_id + data_index_message) =
        static_cast<uint32_t>(point.tree_root_.column_index * num_rows + point.tree_root_.row_index);
    *(container.iter_id + data_index_message) = point.id;
}

void addRawPointToMessage(PointCloud2Iterators& container, int data_index_message, const RawPoint& point)
{
    *(container.iter_x_out + data_index_message) = point.x;
    *(container.iter_y_out + data_index_message) = point.y;
    *(container.iter_z_out + data_index_message) = point.z;
    *(container.iter_f_out + data_index_message) = static_cast<uint32_t>(point.firing_index);
    *(container.iter_i_out + data_index_message) = point.intensity;
    *(container.iter_gpi_out + data_index_message) =
        *reinterpret_cast<const double*>(&point.globally_unique_point_index); // PointCloud2 does not support UINT64

    ros::Time t;
    t.fromNSec(point.stamp);
    *(container.iter_time_sec_out + data_index_message) = t.sec;
    *(container.iter_time_nsec_out + data_index_message) = t.nsec;
}

sensor_msgs::PointCloud2Ptr
evaluationToPointCloud(const std::vector<KittiSegmentationEvaluationPoint>& point_cloud)
{
    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2());
    msg->header.stamp = ros::Time(0, 0); // TODO
    msg->header.frame_id = "velo_link";  // TODO
    msg->width = point_cloud.size();
    msg->height = 1;
    msg->is_bigendian = false;
    msg->is_dense = false;

    sensor_msgs::PointCloud2Modifier output_modifier(*msg);
    output_modifier.setPointCloud2Fields(8,
                                         "x",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "y",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "z",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "has_corresponding_point_in_detection_point_cloud",
                                         1,
                                         sensor_msgs::PointField::UINT8,
                                         "ground_point_evaluation",
                                         1,
                                         sensor_msgs::PointField::UINT8,
                                         "ground_truth_label",
                                         1,
                                         sensor_msgs::PointField::UINT32,
                                         "detection_label",
                                         1,
                                         sensor_msgs::PointField::UINT32,
                                         "false_negative",
                                         1,
                                         sensor_msgs::PointField::UINT8);

    sensor_msgs::PointCloud2Iterator<float> iter_x_out(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y_out(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z_out(*msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_has_corresponding_point_in_detection_point_cloud_out(
        *msg, "has_corresponding_point_in_detection_point_cloud");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_gpe_out(*msg, "ground_point_evaluation");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_gt_label_out(*msg, "ground_truth_label");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_det_label_out(*msg, "detection_label");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_false_negative_out(*msg, "false_negative");

    int32_t i = 0;
    for (const auto& p : point_cloud)
    {
        uint8_t ground_point_evaluation_color = WHITE;
        if (p.ground_point_true_positive)
            ground_point_evaluation_color = GREEN;
        else if (p.ground_point_false_negative)
            ground_point_evaluation_color = YELLOWGREEN;
        else if (p.ground_point_true_negative)
            ground_point_evaluation_color = RED;
        else if (p.ground_point_false_positive)
            ground_point_evaluation_color = DARKRED;

        *(iter_x_out + i) = p.point.x;
        *(iter_y_out + i) = p.point.y;
        *(iter_z_out + i) = p.point.z;
        *(iter_has_corresponding_point_in_detection_point_cloud_out + i) =
            p.has_corresponding_point_in_detection_point_cloud ? WHITE : RED;
        *(iter_gpe_out + i) = ground_point_evaluation_color;
        *(iter_gt_label_out + i) = p.euclidean_clustering_label;
        *(iter_det_label_out + i) = p.detection_label;
        *(iter_false_negative_out + i) = p.euclidean_clustering_label != 0 && p.detection_label == 0 ? RED : WHITE;
        i++;
    }

    return msg;
}

void publish_tf(tf2_ros::TransformBroadcaster& pub, const Eigen::Isometry3d& odom_from_velodyne, uint64_t stamp)
{
    auto msg = tf2::eigenToTransform(odom_from_velodyne);
    msg.header.stamp.fromNSec(stamp);
    msg.header.frame_id = "odom";
    msg.child_frame_id = "velo_link";
    pub.sendTransform(msg);
}

void publish_clock(ros::Publisher& pub, uint64_t stamp)
{
    if (pub.getNumSubscribers() == 0)
        return;

    rosgraph_msgs::Clock msg;
    msg.clock.fromNSec(stamp);
    pub.publish(msg);
}

void publish_ego_robot_bounding_box(uint64_t stamp,
                                    ros::Publisher& pub,
                                    const ContinuousGroundSegmentationConfiguration& config)
{
    if (pub.getNumSubscribers() == 0)
        return;

    visualization_msgs::Marker msg;
    msg.header.frame_id = "velo_link";
    msg.header.stamp.fromNSec(stamp);
    msg.id = 0;
    msg.ns = "ego_robot";
    msg.lifetime = ros::Duration(0, 0);
    msg.type = visualization_msgs::Marker::CUBE;
    msg.color.r = 1;
    msg.color.g = 1;
    msg.color.b = 1;
    msg.color.a = 0.4;
    msg.scale.x = std::abs(config.length_ref_to_rear_end_) + std::abs(config.length_ref_to_front_end_);
    msg.scale.y = std::abs(config.width_ref_to_right_mirror_) + std::abs(config.width_ref_to_left_mirror_);
    msg.scale.z = std::abs(config.height_ref_to_ground_) + std::abs(config.height_ref_to_maximum_);

    // calculate center of bounding box relative to velodyne sensor
    msg.pose.position.x = config.length_ref_to_rear_end_ + msg.scale.x / 2;
    msg.pose.position.y = config.width_ref_to_right_mirror_ + msg.scale.y / 2;
    msg.pose.position.z = config.height_ref_to_ground_ + msg.scale.z / 2;

    // approximately same orientation as velodyne
    msg.pose.orientation.w = 1;

    // ensure that its position within the fixed frame is always updated although the message is just published once
    msg.frame_locked = true;

    pub.publish(msg);
}

} // namespace continuous_clustering