#include <continuous_clustering/ContinuousClusteringConfig.h>
#include <continuous_clustering/clustering/continuous_range_image.hpp>
#include <continuous_clustering/clustering/pipeline_nodes/association.hpp>
#include <continuous_clustering/clustering/pipeline_nodes/cluster_detection.hpp>
#include <continuous_clustering/clustering/pipeline_nodes/coordinate_transformation.hpp>
#include <continuous_clustering/clustering/pipeline_nodes/driver.hpp>
#include <continuous_clustering/clustering/pipeline_nodes/ground_point_segmentation.hpp>
#include <continuous_clustering/clustering/pipeline_nodes/point_collection.hpp>
#include <continuous_clustering/clustering/pipeline_nodes/point_filtering.hpp>
#include <continuous_clustering/clustering/pipeline_nodes/range_image_generation.hpp>
#include <continuous_clustering/clustering/point_types.hpp>
#include <continuous_clustering/ros/generic_points_input.hpp>
#include <continuous_clustering/ros/ouster_input.hpp>
#include <continuous_clustering/ros/ros_utils.hpp>
#include <continuous_clustering/ros/sensor_input.hpp>
#include <continuous_clustering/ros/velodyne_input.hpp>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

namespace continuous_clustering
{

class RosContinuousClustering
{
  public:
    RosContinuousClustering(ros::NodeHandle nh, const ros::NodeHandle& nh_private)
        : nh_(nh), tf_buffer(), tf_listener(tf_buffer, true, ros::TransportHints().tcpNoDelay())
    {
        // obtain ROS specific parameters
        nh_private.param<std::string>("sensor_manufacturer", sensor_manufacturer, "velodyne");
        nh_private.param<std::string>("sensor_frame", sensor_frame, "sensor/lidar/vls128_roof");
        nh_private.param<std::string>("odom_frame", odom_frame, "odom");
        nh_private.param<std::string>("ego_robot_frame", ego_robot_frame, "base_link");
        nh_private.param<bool>("wait_for_tf", wait_for_tf, true);

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

    void reset()
    {
        // reset
        last_update_ = ros::Time(0, 0);
        first_firing_after_reset = true;

        // stop all threads in all pipeline nodes
        range_image_generation_node.stop();
        point_filtering_node.stop();
        coordinate_transformation_node.stop();
        ground_point_segmentation_node.stop();
        association_node.stop();
        cluster_detection_node.stop();
        point_collection_node.stop();

        // reset range image
        range_image =
            std::make_shared<ContinuousRangeImage>(config_.num_rows,
                                                   config_.num_columns,
                                                   config_.num_columns * 10,
                                                   config_.sensor_is_clockwise,
                                                   config_.supplement_inclination_angle_for_nan_cells); // todo

        // tell the pipeline nodes about new range image
        range_image_generation_node.setContinuousRangeImage(range_image);
        point_filtering_node.setContinuousRangeImage(range_image);
        coordinate_transformation_node.setContinuousRangeImage(range_image);
        ground_point_segmentation_node.setContinuousRangeImage(range_image);
        association_node.setContinuousRangeImage(range_image);
        cluster_detection_node.setContinuousRangeImage(range_image);
        point_collection_node.setContinuousRangeImage(range_image);

        // tell the pipeline nodes about their successor
        range_image_generation_node.setConsumerNode(point_filtering_node);
        point_filtering_node.setConsumerNode(coordinate_transformation_node);
        coordinate_transformation_node.setConsumerNode(ground_point_segmentation_node);
        ground_point_segmentation_node.setConsumerNode(association_node);
        association_node.setConsumerNode(cluster_detection_node);
        cluster_detection_node.setConsumerNode(point_collection_node);
        // point_collection_node.setConsumerNode(range_image);

        // reset all pipeline nodes
        range_image_generation_node.reset();
        point_filtering_node.reset();
        coordinate_transformation_node.reset();
        ground_point_segmentation_node.reset();
        association_node.reset();
        cluster_detection_node.reset();
        point_collection_node.reset();

        // coordinate transform node requires transform info for a particular timestamp, which it requests via callback
        coordinate_transformation_node.setRequestTransformOdomFromSensorCallback(
            [this](uint64_t stamp, TransformStatus& status)
            { return onRequestTransformOdomFromSensor(stamp, status); });

        // register callbacks to be notified when columns are finished at particular stages in the pipeline
        range_image_generation_node.setJobFinishedCallback(
            [this](const RangeImageGenerationResult& result)
            { onFinishedColumn(result.column_index, result.column_index, RANGE_IMAGE_GENERATION); });
        ground_point_segmentation_node.setJobFinishedCallback(
            [this](const GroundPointSegmentationResult& result)
            { onFinishedColumn(result.column_index, result.column_index, GROUND_POINT_SEGMENTATION); });
        point_collection_node.setJobFinishedCallback(
            [this](const PointCollectionResult& result)
            {
                // get fully processed columns (this locks a mutex)
                int64_t finished_column_index_start;
                int64_t finished_column_index_end;
                range_image->getColumnsRangeToClear(result.column_index,
                                                    result.outdated_minimum_required_column_indices,
                                                    finished_column_index_start,
                                                    finished_column_index_end);

                // the acquired mutex is important as otherwise this function could be interrupted in-between by another
                // finished point collection job
                onFinishedColumn(finished_column_index_start, finished_column_index_end, CONTINUOUS_CLUSTERING);

                // advance ring buffer (this unlocks above mutex)
                range_image->advanceStartOfRingBuffer(finished_column_index_start, finished_column_index_end);

                // the points were already copied from the range image; therefore, we can publish them after clearance
                for (const auto& cluster_with_stamp : result.clusters_with_stamp)
                    onFinishedCluster(*cluster_with_stamp.first, cluster_with_stamp.second);
            });

        // start all threads
        range_image_generation_node.start();
        point_filtering_node.start();
        coordinate_transformation_node.start();
        ground_point_segmentation_node.start();
        association_node.start();
        cluster_detection_node.start();
        point_collection_node.start();

        // reset ROS tf
        tf_buffer.clear();

        // reset sensor input (todo: right position of reset?)
        sensor_input_->reset();

        // general config todo
        // config_.general.is_single_threaded = config.is_single_threaded;

        applyConfig();
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
            // first firing after reset arrives (todo: check this)
            if (std::abs(dt) > 0.1)
            {
                ROS_WARN_STREAM("Detected jump in time (" << dt << "). Reset continuous clustering.");
                reset();
                return;
            }
        }
        first_firing_after_reset = false;
        last_update_ = stamp_current_firing;

        // insert firing into processing pipeline
        DriverResult next_job;
        next_job.firing = firing;
        range_image_generation_node.enqueueJob(std::move(next_job));

        // publish Firing (just for debugging/visualization)
        if (pub_raw_firings.getNumSubscribers() > 0)
            pub_raw_firings.publish(firingToPointCloud(firing, sensor_frame));
    }

    Eigen::Isometry3d onRequestTransformOdomFromSensor(uint64_t stamp, TransformStatus& status)
    {

        // todo: separate this?
        auto& ref_from_sensor = ground_point_segmentation_node.getConfig().ego_bounding_box.transform_ref_from_sensor;
        if (!ref_from_sensor)
        {
            try
            {
                geometry_msgs::TransformStamped tf =
                    tf_buffer.lookupTransform(ego_robot_frame, sensor_frame, ros::Time());
                ref_from_sensor = tf2::transformToEigen(tf.transform);
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
        auto msg = columnToPointCloud(*range_image,
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

    void applyConfig()
    {
        PointFilteringConfig c1;
        c1.ignore_points_in_chessboard_pattern = config_.ignore_points_in_chessboard_pattern;
        c1.ignore_points_with_too_big_inclination_angle_diff =
            config_.ignore_points_with_too_big_inclination_angle_diff;
        c1.max_distance = static_cast<float>(config_.max_distance);
        c1.min_distance_from_sensor_origin = static_cast<float>(config_.min_distance_from_sensor_origin); // todo
        c1.fog_filtering_enabled = config_.fog_filtering_enabled;
        c1.fog_filtering_intensity_below = config_.fog_filtering_intensity_below;
        c1.fog_filtering_distance_below = static_cast<float>(config_.fog_filtering_distance_below);
        c1.fog_filtering_inclination_above = static_cast<float>(config_.fog_filtering_inclination_above);
        point_filtering_node.setConfig(c1);

        GroundPointSegmentationConfig c2;
        c2.ego_bounding_box = readEgoBoundingBoxFromParameterServer();
        c2.max_slope = static_cast<float>(config_.max_slope);
        c2.first_ring_as_ground_max_allowed_z_diff =
            static_cast<float>(config_.first_ring_as_ground_max_allowed_z_diff);
        c2.first_ring_as_ground_min_allowed_z_diff =
            static_cast<float>(config_.first_ring_as_ground_min_allowed_z_diff);
        c2.last_ground_point_slope_higher_than = static_cast<float>(config_.last_ground_point_slope_higher_than);
        c2.last_ground_point_distance_smaller_than =
            static_cast<float>(config_.last_ground_point_distance_smaller_than);
        c2.ground_because_close_to_last_certain_ground_max_z_diff =
            static_cast<float>(config_.ground_because_close_to_last_certain_ground_max_z_diff);
        c2.ground_because_close_to_last_certain_ground_max_dist_diff =
            static_cast<float>(config_.ground_because_close_to_last_certain_ground_max_dist_diff);
        c2.obstacle_because_next_certain_obstacle_max_dist_diff =
            static_cast<float>(config_.obstacle_because_next_certain_obstacle_max_dist_diff);
        ground_point_segmentation_node.setConfig(c2);

        AssociationConfig c3;
        c3.max_distance = static_cast<float>(config_.max_distance);
        c3.max_steps_in_row = config_.max_steps_in_row;
        c3.max_steps_in_column = config_.max_steps_in_column;
        c3.stop_after_association_enabled = config_.stop_after_association_enabled;
        c3.stop_after_association_min_steps = config_.stop_after_association_min_steps;
        association_node.setConfig(c3);

        PointCollectionConfig c4;
        c4.use_last_point_for_cluster_stamp = config_.use_last_point_for_cluster_stamp;
        point_collection_node.setConfig(c4);
    }

    void callbackReconfigure(ContinuousClusteringConfig& config, uint32_t level)
    {
        // some parameter changes need a hard reset
        bool reset_required = first_reconfigure;
        // if (config_.general.is_single_threaded != config.general.is_single_threaded) todo
        //     reset_required = true;
        if (config_.sensor_is_clockwise != config.sensor_is_clockwise)
            reset_required = true;
        if (config_.num_rows != config.num_rows)
            reset_required = true;
        if (config_.num_columns != config.num_columns)
            reset_required = true;
        if (config_.supplement_inclination_angle_for_nan_cells != config.supplement_inclination_angle_for_nan_cells)
            reset_required = true; // todo: really required?

        // save new config
        config_ = config;
        first_reconfigure = false;

        // only reset if required
        if (reset_required)
            reset();
        else
            applyConfig();
    }

    EgoVehicleBoundingBox readEgoBoundingBoxFromParameterServer()
    {
        EgoVehicleBoundingBox ego_bounding_box;
        bool success = true;

        // get name of ego vehicle
        std::string ego_name;
        success &= nh_.getParam("/vehicles/ego", ego_name);

        // get parameters
        success &= nh_.getParam("/vehicles/" + ego_name + "/geometric/height_ref_to_maximum",
                                ego_bounding_box.height_ref_to_top);
        success &= nh_.getParam("/vehicles/" + ego_name + "/geometric/height_ref_to_ground",
                                ego_bounding_box.height_ref_to_bottom);
        success &= nh_.getParam("/vehicles/" + ego_name + "/geometric/length_ref_to_front_end",
                                ego_bounding_box.length_ref_to_front);
        success &= nh_.getParam("/vehicles/" + ego_name + "/geometric/length_ref_to_rear_end",
                                ego_bounding_box.length_ref_to_back);
        success &= nh_.getParam("/vehicles/" + ego_name + "/geometric/width_ref_to_left_mirror",
                                ego_bounding_box.width_ref_to_left);
        success &= nh_.getParam("/vehicles/" + ego_name + "/geometric/width_ref_to_right_mirror",
                                ego_bounding_box.width_ref_to_right);

        // raise error if something went wrong
        if (!success)
            throw std::runtime_error("Not all vehicle parameters are available");

        return ego_bounding_box;
    }

  private:
    // configuration
    dynamic_reconfigure::Server<ContinuousClusteringConfig> reconfigure_server_;
    ContinuousClusteringConfig config_;
    bool first_reconfigure{true};

    // init ROS specific stuff
    ros::NodeHandle nh_;
    std::shared_ptr<SensorInput> sensor_input_;
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

    // range image
    std::shared_ptr<ContinuousRangeImage> range_image;

    // pipeline nodes
    RangeImageGeneration<DriverResult> range_image_generation_node;
    PointFiltering<RangeImageGenerationResult> point_filtering_node;
    CoordinateTransformation<PointFilteringResult> coordinate_transformation_node;
    GroundPointSegmentation<CoordinateTransformationResult> ground_point_segmentation_node;
    Association<GroundPointSegmentationResult> association_node;
    ClusterDetection<AssociationResult> cluster_detection_node;
    PointCollection<ClusterDetectionResult> point_collection_node;
};

} // namespace continuous_clustering

int main(int argc, char** argv)
{
    ros::init(argc, argv, "continuous_clustering");

    continuous_clustering::RosContinuousClustering node(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();

    return 0;
}
