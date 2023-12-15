#include <continuous_clustering/ContinuousClusteringConfig.h>
#include <continuous_clustering/ros/generic_points_input.hpp>
#include <continuous_clustering/ros/ouster_input.hpp>
#include <continuous_clustering/ros/ros_transform_synchronizer.hpp>
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
    RosContinuousClustering(ros::NodeHandle nh, const ros::NodeHandle& nh_private) : nh_(nh), clustering_()
    {
        // obtain ROS specific parameters
        nh_private.param<std::string>("sensor_manufacturer", sensor_manufacturer, "velodyne");
        nh_private.param<std::string>("sensor_frame", sensor_frame, "sensor/lidar/vls128_roof");
        nh_private.param<std::string>("odom_frame", odom_frame, "odom");
        nh_private.param<std::string>("ego_robot_frame", ego_robot_frame, "base_link");
        nh_private.param<bool>("wait_for_tf", wait_for_tf, true);

        // add callbacks to clustering
        clustering_.setFinishedColumnCallback(
            [this](int64_t from_global_column_index, int64_t to_global_column_index, bool ground_points_only)
            { onFinishedColumn(from_global_column_index, to_global_column_index, ground_points_only); });
        clustering_.setFinishedClusterCallback([this](const std::vector<Point>& cluster_points, uint64_t stamp_cluster)
                                               { onFinishedCluster(cluster_points, stamp_cluster); });

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
        pub_ground_point_segmentation =
            nh.advertise<sensor_msgs::PointCloud2>("continuous_ground_point_segmentation", 1000);
        pub_instance_segmentation = nh.advertise<sensor_msgs::PointCloud2>("continuous_instance_segmentation", 1000);
        pub_clusters = nh.advertise<sensor_msgs::PointCloud2>("continuous_clusters", 1000);

        // init TF synchronizer
        tf_synchronizer.setCallback(&RosContinuousClustering::onNewFiringsWithTfCallback, this);

        // init dynamic reconfigure
        reconfigure_server_.setCallback([this](ContinuousClusteringConfig& config, uint32_t level)
                                        { callbackReconfigure(config, level); });
    }

    void reset(int num_rows)
    {
        // reset
        last_update_ = ros::Time(0, 0);
        first_firing_after_reset = true;

        // reset tf synchronizer
        tf_synchronizer.reset(odom_frame, sensor_frame);
        tf_synchronizer.waitForTransform(wait_for_tf);

        // reset clustering
        clustering_.reset(num_rows);

        // clear sensor input
        sensor_input_->reset();
    }

    void onNewFiringsCallback(const RawPoints::ConstPtr& firing)
    {
        // get ROS timestamp
        ros::Time stamp_current_firing;
        stamp_current_firing.fromNSec(firing->stamp);

        // handle time jumps and bad start condition (TODO: put it into library?)
        double dt = (stamp_current_firing - last_update_).toSec();
        if (clustering_.resetRequired() || (!first_firing_after_reset && std::abs(dt) > 0.1))
        {
            ROS_WARN_STREAM("Detected jump in time ("
                            << dt << ") or bad start condition was encountered. Reset continuous clustering.");
            reset(static_cast<int>(firing->points.size()));
            return;
        }
        first_firing_after_reset = false;
        last_update_ = stamp_current_firing;

        // wait for transforms
        tf_synchronizer.addMessage(stamp_current_firing, firing);

        // publish Firing (just for debugging/visualization)
        if (pub_raw_firings.getNumSubscribers() > 0)
            pub_raw_firings.publish(firingToPointCloud(firing, sensor_frame));

        // make statistics of workload for debugging
        clustering_.recordJobQueueWorkload(sensor_input_->dataCount());
    }

    void onNewFiringsWithTfCallback(const RawPoints::ConstPtr& firing,
                                    const geometry_msgs::TransformStamped& odom_from_sensor)
    {
        // get ego robot frame from sensor frame in order to transform points from sensor (odom) into vehicle
        // coordinates (important to find points on roof, etc.)
        if (!clustering_.hasTransformRobotFrameFromSensorFrame())
        {
            try
            {
                geometry_msgs::TransformStamped tf =
                    tf_synchronizer.getTfBuffer()->lookupTransform(ego_robot_frame, sensor_frame, ros::Time());
                clustering_.setTransformRobotFrameFromSensorFrame(tf2::transformToEigen(tf.transform));
            }
            catch (tf2::TransformException& ex)
            {
                return;
            }
        }

        clustering_.addFiring(firing, tf2::transformToEigen(odom_from_sensor));
    }

    void onFinishedCluster(const std::vector<Point>& cluster_points, uint64_t stamp_cluster)
    {
        pub_clusters.publish(clusterToPointCloud(cluster_points, clustering_.num_rows_, stamp_cluster, odom_frame));
    }

    void onFinishedColumn(int64_t from_global_column_index, int64_t to_global_column_index, bool ground_points_only)
    {
        ProcessingStage stage = ground_points_only ? GROUND_POINT_SEGMENTATION : CONTINUOUS_CLUSTERING;
        auto msg = columnToPointCloud(clustering_, from_global_column_index, to_global_column_index, odom_frame, stage);
        ros::Publisher* pub = ground_points_only ? &pub_ground_point_segmentation : &pub_instance_segmentation;
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

        // config for range image
        config_.range_image.sensor_is_clockwise = config.sensor_is_clockwise;
        config_.range_image.num_columns = config.num_columns;
        config_.range_image.supplement_inclination_angle_for_nan_cells =
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
    RosTransformSynchronizer<RawPoints> tf_synchronizer;
    std::shared_ptr<SensorInput> sensor_input_;
    ros::Subscriber sub_terrain;
    ros::Publisher pub_raw_firings;
    ros::Publisher pub_ground_point_segmentation;
    ros::Publisher pub_instance_segmentation;
    ros::Publisher pub_clusters;
    ros::Time last_update_{0, 0};
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
