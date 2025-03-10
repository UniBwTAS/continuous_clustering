#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/Marker.h>

#include <continuous_clustering/ros/ros_utils.hpp>

namespace continuous_clustering
{

sensor_msgs::PointCloud2Ptr clusterToPointCloud(const RangeImage& range_image,
                                                const std::vector<uint64_t>& cluster_pixel_idxs,
                                                uint64_t stamp_cluster,
                                                const std::string& frame_id)
{
    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header.stamp.fromNSec(stamp_cluster);
    msg->header.frame_id = frame_id;
    msg->width = cluster_pixel_idxs.size();
    msg->height = 1;

    PointCloud2Iterators container = prepareMessageAndCreateIterators(*msg, CONTINUOUS_CLUSTERING);

    int data_index_message = 0;
    for (uint64_t pixel_idx : cluster_pixel_idxs)
    {
        addPointToMessage(container, data_index_message, pixel_idx, range_image, CONTINUOUS_CLUSTERING);
        data_index_message++;
    }

    return msg;
}

sensor_msgs::PointCloud2Ptr columnToPointCloud(const RangeImage& range_image,
                                               int64_t from_monot_col_idx,
                                               int64_t to_monot_col_idx,
                                               const std::string& frame_id,
                                               ProcessingStage fill_fields_up_to_stage)
{
    int num_columns_to_publish = static_cast<int>(to_monot_col_idx - from_monot_col_idx) + 1;
    if (num_columns_to_publish <= 0)
        return nullptr;

    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header.frame_id = frame_id;
    msg->width = num_columns_to_publish;
    msg->height = range_image.height;

    PointCloud2Iterators container = prepareMessageAndCreateIterators(*msg, fill_fields_up_to_stage);

    // keep track of minimum point stamp in this message
    uint64_t minimum_point_stamp = std::numeric_limits<uint64_t>::max();

    for (int message_column_index = 0; message_column_index < msg->width; ++message_column_index)
    {
        uint16_t ring_buf_col_idx = range_image.fromMonotColIdx(from_monot_col_idx + message_column_index);
        for (int row_index = 0; row_index < range_image.height; ++row_index)
        {
            uint64_t pixel_idx = range_image.getIndex(ring_buf_col_idx, row_index);
            int data_index_message = row_index * static_cast<int>(msg->width) + message_column_index;
            addPointToMessage(container, data_index_message, pixel_idx, range_image, fill_fields_up_to_stage);

            if (range_image.stamp_ns[pixel_idx] != 0 && range_image.stamp_ns[pixel_idx] < minimum_point_stamp)
                minimum_point_stamp = range_image.stamp_ns[pixel_idx];
        }
    }

    if (minimum_point_stamp != std::numeric_limits<uint64_t>::max())
        msg->header.stamp.fromNSec(minimum_point_stamp);
    /*else
        ROS_WARN_STREAM("This column had no timestamps. Unable to publish message with timestamp. Local Column Index: "
                        << from_monot_col_idx % clustering.num_columns_rot_ << ", " << num_columns_to_publish);*/

    return msg;
}

sensor_msgs::PointCloud2Ptr firingToPointCloud(const RawPoints::ConstPtr& firing, const std::string& frame_id)
{
    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header.frame_id = frame_id;
    msg->width = 1;
    msg->height = firing->points.size();

    PointCloud2Iterators container = prepareMessageAndCreateIterators(*msg, RAW_POINT);

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

PointCloud2Iterators prepareMessageAndCreateIterators(sensor_msgs::PointCloud2& msg,
                                                      ProcessingStage fill_fields_up_to_stage)
{
    msg.is_bigendian = false;
    msg.is_dense = false;

    int up_to_field = 25;
    if (fill_fields_up_to_stage == RAW_POINT)
        up_to_field = 8;
    else if (fill_fields_up_to_stage == RANGE_IMAGE_GENERATION)
        up_to_field = 15;
    else if (fill_fields_up_to_stage == GROUND_POINT_SEGMENTATION)
        up_to_field = 19;
    else if (fill_fields_up_to_stage == CONTINUOUS_CLUSTERING)
        up_to_field = 22;

    // Some point fields below should be actually UINT64. Unfortunately, this type is not available for a PointCloud2
    // message: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointField.html. Therefore, we use FLOAT64 which
    // is able to store higher integers than UINT32 (2^52 vs 2^32); these fields are marked with (*)

    sensor_msgs::PointCloud2Modifier output_modifier(msg);
    output_modifier.setPointCloud2Fields(up_to_field,
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
                                         sensor_msgs::PointField::FLOAT64, // (*)
                                         "intensity",
                                         1,
                                         sensor_msgs::PointField::UINT8,
                                         "globally_unique_point_index",
                                         1,
                                         sensor_msgs::PointField::FLOAT64, // (*)
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
                                         "elevation_angle",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "monot_azimuth_angle",
                                         1,
                                         sensor_msgs::PointField::FLOAT64,
                                         "monot_col_idx",
                                         1,
                                         sensor_msgs::PointField::FLOAT64, // (*)
                                         "col_idx",
                                         1,
                                         sensor_msgs::PointField::UINT16,
                                         "row_index",
                                         1,
                                         sensor_msgs::PointField::UINT16,
                                         "ground_point_label",
                                         1,
                                         sensor_msgs::PointField::UINT8,
                                         "debug_ground_point_label",
                                         1,
                                         sensor_msgs::PointField::UINT8,
                                         "height_over_ground",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "ignore_for_clustering",
                                         1,
                                         sensor_msgs::PointField::UINT8,
                                         "finished_at_monot_azimuth_angle",
                                         1,
                                         sensor_msgs::PointField::UINT16,
                                         "number_of_visited_neighbors",
                                         1,
                                         sensor_msgs::PointField::UINT32,
                                         "id",
                                         1,
                                         sensor_msgs::PointField::FLOAT64); // (*)

    PointCloud2Iterators iterators;
    iterators.iter_x_out = {msg, "x"};
    iterators.iter_y_out = {msg, "y"};
    iterators.iter_z_out = {msg, "z"};
    iterators.iter_f_out = {msg, "firing_index"};
    iterators.iter_i_out = {msg, "intensity"};
    iterators.iter_gpi_out = {msg, "globally_unique_point_index"};
    iterators.iter_time_sec_out = {msg, "time_sec"};
    iterators.iter_time_nsec_out = {msg, "time_nsec"};
    if (fill_fields_up_to_stage == RAW_POINT)
        return iterators;
    iterators.iter_d_out = {msg, "distance"};
    iterators.iter_a_out = {msg, "azimuth_angle"};
    iterators.iter_ia_out = {msg, "elevation_angle"};
    iterators.iter_ca_out = {msg, "monot_azimuth_angle"};
    iterators.iter_gc_out = {msg, "monot_col_idx"};
    iterators.iter_lc_out = {msg, "col_idx"};
    iterators.iter_r_out = {msg, "row_index"};
    if (fill_fields_up_to_stage == RANGE_IMAGE_GENERATION)
        return iterators;
    iterators.iter_gp_label_out = {msg, "ground_point_label"};
    iterators.iter_dbg_gp_label_out = {msg, "debug_ground_point_label"};
    iterators.iter_height_over_ground_out = {msg, "height_over_ground"};
    iterators.iter_ignore_for_clustering_out = {msg, "ignore_for_clustering"};
    if (fill_fields_up_to_stage == GROUND_POINT_SEGMENTATION)
        return iterators;
    iterators.iter_finished_at_azimuth_angle = {msg, "finished_at_monot_azimuth_angle"};
    iterators.iter_number_of_visited_neighbors = {msg, "number_of_visited_neighbors"};
    iterators.iter_id = {msg, "id"};
    return iterators;
}

void addPointToMessage(PointCloud2Iterators& container,
                       int data_index_message,
                       int64_t pixel_idx,
                       const continuous_clustering::RangeImage& range_image,
                       ProcessingStage fill_fields_up_to_stage)
{
    ros::Time stamp;
    stamp.fromNSec(range_image.stamp_ns[pixel_idx]);

    // Some point fields below should be actually UINT64. Unfortunately, this type is not available for a PointCloud2
    // message: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointField.html. Therefore, we use FLOAT64 which
    // is able to store higher integers than UINT32 (2^52 vs 2^32); these fields are marked with (*)

    // raw point
    *(*container.iter_x_out + data_index_message) = range_image.x[pixel_idx];
    *(*container.iter_y_out + data_index_message) = range_image.y[pixel_idx];
    *(*container.iter_z_out + data_index_message) = range_image.z[pixel_idx];
    *(*container.iter_f_out + data_index_message) = static_cast<double>(range_image.firing_idx[pixel_idx]); // (*)
    *(*container.iter_i_out + data_index_message) = range_image.intensity[pixel_idx];
    *(*container.iter_gpi_out + data_index_message) =
        static_cast<double>(range_image.globally_unique_point_index[pixel_idx]); // (*)
    *(*container.iter_time_sec_out + data_index_message) = stamp.sec;
    *(*container.iter_time_nsec_out + data_index_message) = stamp.nsec;
    if (fill_fields_up_to_stage == RAW_POINT)
        return;

    // range image generation
    *(*container.iter_d_out + data_index_message) = range_image.distance[pixel_idx];
    *(*container.iter_a_out + data_index_message) = range_image.azimuth_angle[pixel_idx];
    *(*container.iter_ia_out + data_index_message) = range_image.elevation_angle[pixel_idx];
    *(*container.iter_ca_out + data_index_message) = range_image.monot_azimuth_angle[pixel_idx];
    *(*container.iter_gc_out + data_index_message) = static_cast<double>(range_image.monot_col_idx[pixel_idx]); // (*)
    *(*container.iter_lc_out + data_index_message) = range_image.col_idx[pixel_idx];
    *(*container.iter_r_out + data_index_message) = range_image.row_idx[pixel_idx];
    if (fill_fields_up_to_stage == RANGE_IMAGE_GENERATION)
        return;

    // ground point segmentation
    *(*container.iter_gp_label_out + data_index_message) = range_image.ground_point_label[pixel_idx];
    *(*container.iter_dbg_gp_label_out + data_index_message) = range_image.debug_ground_point_label[pixel_idx];
    *(*container.iter_height_over_ground_out + data_index_message) = range_image.height_over_ground[pixel_idx];
    *(*container.iter_ignore_for_clustering_out + data_index_message) =
        range_image.is_ignored[pixel_idx] ? BLUE : ORANGE;
    if (fill_fields_up_to_stage == GROUND_POINT_SEGMENTATION)
        return;

    // continuous clustering
    *(*container.iter_finished_at_azimuth_angle + data_index_message) =
        range_image.finished_at_monot_azimuth_angle[pixel_idx];
    *(*container.iter_number_of_visited_neighbors + data_index_message) =
        range_image.number_of_visited_neighbors[pixel_idx];
    *(*container.iter_id + data_index_message) = static_cast<double>(range_image.id[pixel_idx]); // (*)
}

void addRawPointToMessage(PointCloud2Iterators& container, int data_index_message, const RawPoint& point)
{
    // Some point fields below should be actually UINT64. Unfortunately, this type is not available for a PointCloud2
    // message: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointField.html. Therefore, we use FLOAT64 which
    // is able to store higher integers than UINT32 (2^52 vs 2^32); these fields are marked with (*)

    *(*container.iter_x_out + data_index_message) = point.x;
    *(*container.iter_y_out + data_index_message) = point.y;
    *(*container.iter_z_out + data_index_message) = point.z;
    *(*container.iter_f_out + data_index_message) = static_cast<double>(point.firing_index); // (*)
    *(*container.iter_i_out + data_index_message) = point.intensity;
    *(*container.iter_gpi_out + data_index_message) = static_cast<double>(point.globally_unique_point_index); // (*)

    ros::Time t;
    t.fromNSec(point.stamp);
    *(*container.iter_time_sec_out + data_index_message) = t.sec;
    *(*container.iter_time_nsec_out + data_index_message) = t.nsec;
}

sensor_msgs::PointCloud2Ptr evaluationToPointCloud(const std::vector<KittiSegmentationEvaluationPoint>& point_cloud)
{
    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2());
    msg->header.stamp = ros::Time(0, 0);
    msg->header.frame_id = "velo_link";
    msg->width = point_cloud.size();
    msg->height = 1;
    msg->is_bigendian = false;
    msg->is_dense = false;

    sensor_msgs::PointCloud2Modifier output_modifier(*msg);
    output_modifier.setPointCloud2Fields(10,
                                         "x",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "y",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "z",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "semantic_label",
                                         1,
                                         sensor_msgs::PointField::UINT16,
                                         "instance_label",
                                         1,
                                         sensor_msgs::PointField::UINT16,
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
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_semantic_label_out(*msg, "semantic_label");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_instance_label_out(*msg, "instance_label");
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
        *(iter_semantic_label_out + i) = p.point.semantic_label;
        *(iter_instance_label_out + i) = p.point.instance_label;
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