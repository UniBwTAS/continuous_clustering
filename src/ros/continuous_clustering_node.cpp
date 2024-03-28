#include <continuous_clustering/ContinuousClusteringConfig.h>
#include <continuous_clustering/ros/generic_points_input.hpp>
#include <continuous_clustering/ros/ouster_input.hpp>
#include <continuous_clustering/ros/ros_utils.hpp>
#include <continuous_clustering/ros/sensor_input.hpp>
#include <continuous_clustering/ros/velodyne_input.hpp>
#include <dynamic_reconfigure/server.h>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <continuous_clustering/clustering/continuous_clustering.hpp>

namespace continuous_clustering
{

class RosContinuousClustering
{
  public:
    RosContinuousClustering(ros::NodeHandle nh, const ros::NodeHandle& nh_private)
        : nh_(nh), clustering_(), tf_buffer(), tf_listener(tf_buffer, true, ros::TransportHints().tcpNoDelay())
    {
        // obtain ROS specific parameters
        nh_private.param<std::string>("sensor_manufacturer", sensor_manufacturer, "velodyne");
        nh_private.param<std::string>("sensor_frame", sensor_frame, "sensor/lidar/vls128_roof");
        nh_private.param<std::string>("odom_frame", odom_frame, "odom");
        nh_private.param<std::string>("ego_robot_frame", ego_robot_frame, "base_link");
        nh_private.param<bool>("wait_for_tf", wait_for_tf, true);

        // add callbacks to clustering
        clustering_.setFinishedColumnCallback(
            [this](int64_t from_global_column_index, int64_t to_global_column_index, ProcessingStage stage)
            { onFinishedColumn(from_global_column_index, to_global_column_index, stage); });
        clustering_.setFinishedClusterCallback([this](const std::vector<Point>& cluster_points, uint64_t stamp_cluster)
                                               { onFinishedCluster(cluster_points, stamp_cluster); });
        clustering_.setRequestTransformOdomFromSensorCallback(
            [this](int64_t stamp, TransformStatus& status) { return onRequestTransformOdomFromSensor(stamp, status); });

        // use desired sensor input
        if (sensor_manufacturer == "velodyne")
            sensor_input_.reset(new VelodyneInput(nh, nh_private));
        else if (sensor_manufacturer == "ouster")
            sensor_input_.reset(new OusterInput(nh, nh_private));
        else if (sensor_manufacturer == "generic_points")
            sensor_input_.reset(new GenericPointsInput(nh, nh_private));
        else
            throw std::runtime_error("Unknown manufacturer: " + sensor_manufacturer);
        sensor_input_->subscribe();
        sensor_input_->addOnNewFiringCallback([this](const RawPoints::ConstPtr& firing)
                                              { onNewFiringsCallback(firing); });

        // obtain ego vehicle parameters (important to remove points on ego vehicle during ground point segmentation)
        bool success = true;
        std::string ego_name;
        success &= nh.getParam("/vehicles/ego", ego_name);
        success &= nh.getParam("/vehicles/" + ego_name + "/geometric/height_ref_to_maximum",
                               config_.ground_segmentation.height_ref_to_maximum_);
        success &= nh.getParam("/vehicles/" + ego_name + "/geometric/height_ref_to_ground",
                               config_.ground_segmentation.height_ref_to_ground_);
        success &= nh.getParam("/vehicles/" + ego_name + "/geometric/length_ref_to_front_end",
                               config_.ground_segmentation.length_ref_to_front_end_);
        success &= nh.getParam("/vehicles/" + ego_name + "/geometric/length_ref_to_rear_end",
                               config_.ground_segmentation.length_ref_to_rear_end_);
        success &= nh.getParam("/vehicles/" + ego_name + "/geometric/width_ref_to_left_mirror",
                               config_.ground_segmentation.width_ref_to_left_mirror_);
        success &= nh.getParam("/vehicles/" + ego_name + "/geometric/width_ref_to_right_mirror",
                               config_.ground_segmentation.width_ref_to_right_mirror_);
        if (!success)
            throw std::runtime_error("Not all vehicle parameters are available");

        // init publishers
        pub_raw_firings = nh.advertise<sensor_msgs::PointCloud2>("raw_firings", 1000);
        pub_range_image_generation = nh.advertise<sensor_msgs::PointCloud2>("continuous_range_image_generation", 1000);
        pub_ground_point_segmentation =
            nh.advertise<sensor_msgs::PointCloud2>("continuous_ground_point_segmentation", 1000);
        pub_instance_segmentation = nh.advertise<sensor_msgs::PointCloud2>("continuous_instance_segmentation", 1000);
        pub_clusters = nh.advertise<sensor_msgs::PointCloud2>("continuous_clusters", 1000);

        // init dynamic reconfigure
        reconfigure_server_.setCallback([this](ContinuousClusteringConfig& config, uint32_t level)
                                        { callbackReconfigure(config, level); });
    }

    void reset(int num_rows)
    {
        // reset
        last_update_ = ros::Time(0, 0);
        first_firing_after_reset = true;

        // reset clustering
        clustering_.reset(num_rows);

        // reset tf synchronizer
        tf_buffer.clear();

        // clear sensor input
        sensor_input_->reset();
    }

    void onNewFiringsCallback(const RawPoints::ConstPtr& firing)
    {
        // get ROS timestamp
        ros::Time stamp_current_firing;
        stamp_current_firing.fromNSec(firing->stamp);

        // handle time jumps and bad start condition
        double dt = (stamp_current_firing - last_update_).toSec();
        if (!first_firing_after_reset)
        {
            // resetting takes some time (often more than 0.1s) therefore we do not want to reset again when the
            // first firing after reset arrives

            if (clustering_.resetRequired())
            {
                ROS_WARN_STREAM("Detected reconfiguration or bad start condition. Reset continuous clustering.");
                reset(static_cast<int>(firing->points.size()));
                return;
            }
            else if (std::abs(dt) > 0.1)
            {
                ROS_WARN_STREAM("Detected jump in time (" << dt << "). Reset continuous clustering.");
                reset(static_cast<int>(firing->points.size()));
                return;
            }
        }
        first_firing_after_reset = false;
        last_update_ = stamp_current_firing;

        // wait for transforms
        clustering_.addFiring(firing);

        // publish Firing (just for debugging/visualization)
        if (pub_raw_firings.getNumSubscribers() > 0)
            pub_raw_firings.publish(firingToPointCloud(firing, sensor_frame));

        // make statistics of workload for debugging
        clustering_.recordJobQueueWorkload(sensor_input_->dataCount());
    }

    Eigen::Isometry3d onRequestTransformOdomFromSensor(uint64_t stamp, TransformStatus& status)
    {

        // todo: separate this?
        if (!clustering_.hasTransformRobotFrameFromSensorFrame())
        {
            try
            {
                geometry_msgs::TransformStamped tf =
                    tf_buffer.lookupTransform(ego_robot_frame, sensor_frame, ros::Time());
                clustering_.setTransformRobotFrameFromSensorFrame(tf2::transformToEigen(tf.transform));
            }
            catch (tf2::TransformException& ex)
            {
                status = TRANSFORM_ERROR;
                return {};
            }
        }

        geometry_msgs::TransformStamped tf_latest{}, tf_at_stamp{};
        try
        {
            ros::Time t{0, 0};
            tf_latest = tf_buffer.lookupTransform(odom_frame, sensor_frame, t);
            if (!wait_for_tf)
            {
                status = TRANSFORM_SUCCESS;
                return tf2::transformToEigen(tf_latest.transform);
            }
            else
            {
                t.fromNSec(stamp);
                tf_at_stamp = tf_buffer.lookupTransform(odom_frame, sensor_frame, t, ros::Duration(0.005));
                status = TRANSFORM_SUCCESS;
                return tf2::transformToEigen(tf_at_stamp.transform);
            }
        }
        catch (tf2::TransformException& ex)
        {
            if (!tf_latest.header.stamp.isZero() && stamp > tf_latest.header.stamp.toNSec())
                status = TRANSFORM_NOT_AVAILABLE_YET;
            else
                status = TRANSFORM_ERROR;
            return {};
        }
    }

    void onFinishedCluster(const std::vector<Point>& cluster_points, uint64_t stamp_cluster)
    {
        pub_clusters.publish(clusterToPointCloud(cluster_points, stamp_cluster, odom_frame));
    }

    void onFinishedColumn(int64_t from_global_column_index, int64_t to_global_column_index, ProcessingStage stage)
    {
        auto msg = columnToPointCloud(clustering_,
                                      from_global_column_index,
                                      to_global_column_index,
                                      stage == RANGE_IMAGE_GENERATION ? sensor_frame : odom_frame,
                                      stage);
        ros::Publisher* pub;
        switch (stage)
        {
            case RANGE_IMAGE_GENERATION:
                pub = &pub_range_image_generation;
                break;
            case GROUND_POINT_SEGMENTATION:
                pub = &pub_ground_point_segmentation;
                break;
            case CONTINUOUS_CLUSTERING:
                pub = &pub_instance_segmentation;
                break;
            default:
                throw std::runtime_error("Invalid stage: " + std::to_string(stage));
        }
        if (msg && pub->getNumSubscribers() > 0)
            pub->publish(msg);
    }

    void callbackReconfigure(ContinuousClusteringConfig& config, uint32_t level)
    {
        // check parameter changes that need additional handling
        if (!config_.ground_segmentation.use_terrain && config.use_terrain)
            sub_terrain = nh_.subscribe("terrain", 1, &RosContinuousClustering::onNewTerrainCallback, this);
        else if (config_.ground_segmentation.use_terrain && !config.use_terrain)
            sub_terrain.shutdown();

        // general config
        config_.general.is_single_threaded = config.is_single_threaded;

        // config for range image
        config_.range_image.sensor_is_clockwise = config.sensor_is_clockwise;
        config_.range_image.num_columns = config.num_columns;
        config_.range_image.interpolate_inclination_angle_for_nan_cells =
            config.supplement_inclination_angle_for_nan_cells;

        // config for ground segmentation
        config_.ground_segmentation.max_slope = static_cast<float>(config.max_slope);
        config_.ground_segmentation.first_ring_as_ground_max_allowed_z_diff =
            static_cast<float>(config.first_ring_as_ground_max_allowed_z_diff);
        config_.ground_segmentation.first_ring_as_ground_min_allowed_z_diff =
            static_cast<float>(config.first_ring_as_ground_min_allowed_z_diff);
        config_.ground_segmentation.last_ground_point_slope_higher_than =
            static_cast<float>(config.last_ground_point_slope_higher_than);
        config_.ground_segmentation.last_ground_point_distance_smaller_than =
            static_cast<float>(config.last_ground_point_distance_smaller_than);
        config_.ground_segmentation.ground_because_close_to_last_certain_ground_max_z_diff =
            static_cast<float>(config.ground_because_close_to_last_certain_ground_max_z_diff);
        config_.ground_segmentation.ground_because_close_to_last_certain_ground_max_dist_diff =
            static_cast<float>(config.ground_because_close_to_last_certain_ground_max_dist_diff);
        config_.ground_segmentation.obstacle_because_next_certain_obstacle_max_dist_diff =
            static_cast<float>(config.obstacle_because_next_certain_obstacle_max_dist_diff);
        config_.ground_segmentation.use_terrain = config.use_terrain;
        config_.ground_segmentation.terrain_max_allowed_z_diff = static_cast<float>(config.terrain_max_allowed_z_diff);
        config_.ground_segmentation.fog_filtering_enabled = config.fog_filtering_enabled;
        config_.ground_segmentation.fog_filtering_intensity_below = config.fog_filtering_intensity_below;
        config_.ground_segmentation.fog_filtering_distance_below =
            static_cast<float>(config.fog_filtering_distance_below);
        config_.ground_segmentation.fog_filtering_inclination_above =
            static_cast<float>(config.fog_filtering_inclination_above);

        // config for clustering
        config_.clustering.max_distance = static_cast<float>(config.max_distance);
        config_.clustering.max_steps_in_row = config.max_steps_in_row;
        config_.clustering.max_steps_in_column = config.max_steps_in_column;
        config_.clustering.stop_after_association_enabled = config.stop_after_association_enabled;
        config_.clustering.stop_after_association_min_steps = config.stop_after_association_min_steps;
        config_.clustering.ignore_points_in_chessboard_pattern = config.ignore_points_in_chessboard_pattern;
        config_.clustering.ignore_points_with_too_big_inclination_angle_diff =
            config.ignore_points_with_too_big_inclination_angle_diff;
        config_.clustering.use_last_point_for_cluster_stamp = config.use_last_point_for_cluster_stamp;

        clustering_.setConfiguration(config_);
    }

    void onNewTerrainCallback(const grid_map_msgs::GridMap::ConstPtr& msg)
    {
        last_terrain_msg_ = msg;
        // TODO: Add it!
    }

  private:
    ContinuousClustering clustering_;

    // configuration
    dynamic_reconfigure::Server<ContinuousClusteringConfig> reconfigure_server_;
    Configuration config_;

    // init ROS specific stuff
    ros::NodeHandle nh_;
    std::shared_ptr<SensorInput> sensor_input_;
    ros::Subscriber sub_terrain;
    ros::Publisher pub_raw_firings;
    ros::Publisher pub_range_image_generation;
    ros::Publisher pub_ground_point_segmentation;
    ros::Publisher pub_instance_segmentation;
    ros::Publisher pub_clusters;
    ros::Time last_update_{0, 0};
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    bool first_firing_after_reset{false};

    std::string sensor_manufacturer;
    std::string sensor_frame;
    std::string odom_frame;
    std::string ego_robot_frame;
    bool wait_for_tf{true};

    grid_map_msgs::GridMap::ConstPtr last_terrain_msg_;
};

} // namespace continuous_clustering

int main(int argc, char** argv)
{
    ros::init(argc, argv, "continuous_clustering");

    continuous_clustering::RosContinuousClustering node(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();

    return 0;
}
