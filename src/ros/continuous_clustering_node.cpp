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
#include <continuous_clustering/ros/generic_points_driver.hpp>
#include <continuous_clustering/ros/ouster_driver.hpp>
#include <continuous_clustering/ros/ros_utils.hpp>
#include <continuous_clustering/ros/velodyne_driver.hpp>
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
        : nh_(nh), nh_private(nh_private), tf_buffer(), tf_listener(tf_buffer, true, ros::TransportHints().tcpNoDelay())
    {
        // init publishers
        pub_raw_firings = nh.advertise<sensor_msgs::PointCloud2>("raw_firings", 1000);
        pub_range_image_generation = nh.advertise<sensor_msgs::PointCloud2>("continuous_range_image_generation", 1000);
        pub_ground_point_segmentation =
            nh.advertise<sensor_msgs::PointCloud2>("continuous_ground_point_segmentation", 1000);
        pub_instance_segmentation = nh.advertise<sensor_msgs::PointCloud2>("continuous_instance_segmentation", 1000);
        pub_clusters = nh.advertise<sensor_msgs::PointCloud2>("continuous_clusters", 1000);
    }

    void run()
    {
        while (ros::ok())
        {
            /*if (config_->sensor_manufacturer == "velodyne")
            {
                runWithDriverUntilReset();
            }
            else if (config_->sensor_manufacturer == "ouster")
            {
                OusterDriver driver{nh_private};
                runWithDriverUntilReset(driver);
            }
            else if (config_->sensor_manufacturer == "generic_points")
            {
                GenericPointsDriver driver{nh_private};
                runWithDriverUntilReset(driver);
            }
            else
            {
                throw std::runtime_error("Unknown manufacturer: " + config_->sensor_manufacturer);
            }*/

            runWithDriverUntilReset();
        }
    }

    void runWithDriverUntilReset()
    {
        bool reset_required{false};

        // setup pipeline
        auto driver = std::make_shared<VelodyneDriver>(nh_private);
        auto range_image_generation = driver->createSuccessor<RangeImageGeneration>();
        auto point_filtering = range_image_generation->createSuccessor<PointFiltering>();
        auto coordinate_transformation = point_filtering->createSuccessor<CoordinateTransformation>();
        auto ground_point_segmentation = coordinate_transformation->createSuccessor<GroundPointSegmentation>();
        auto association = ground_point_segmentation->createSuccessor<Association>();
        auto cluster_detection = association->createSuccessor<ClusterDetection>();
        auto point_collection = cluster_detection->createSuccessor<PointCollection>();

        // register ROS dynamic reconfigure callback
        reconfigure_server.setCallback(
            [&](ContinuousClusteringConfig& new_conf, uint32_t level)
            {
                // required for ground point segmentation
                Eigen::Isometry3d tf = waitForRobotFromSensorTransform(new_conf.ego_robot_frame, new_conf.sensor_frame);
                EgoVehicleBoundingBox box = readEgoBoundingBoxFromParameterServer(nh_);
                box.transform_ref_from_sensor = tf;

                reset_required = false;
                // reset_required |= range_image_generation.setConfig(toRangeImageGeneration(config));
                reset_required |= point_filtering->setConfig(toPointFilteringConfig(new_conf));
                reset_required |= ground_point_segmentation->setConfig(toGroundPointSegmentationConfig(new_conf, box));
                reset_required |= association->setConfig(toAssociationConfig(new_conf));
                reset_required |= point_collection->setConfig(toPointCollectionConfig(new_conf));

                // todo
                if (prev_conf)
                {
                    if (prev_conf->sensor_manufacturer != new_conf.sensor_manufacturer)
                        reset_required = true;
                    if (prev_conf->sensor_is_clockwise != new_conf.sensor_is_clockwise)
                        reset_required = true;
                    if (prev_conf->num_rows != new_conf.num_rows)
                        reset_required = true;
                    if (prev_conf->num_columns != new_conf.num_columns)
                        reset_required = true;
                }
                else
                    reset_required = false;

                prev_conf = new_conf;
            });

        // wait until first dynamic reconfigure callback arrived
        while (!prev_conf)
            ros::spinOnce();

        // todo reset required sensor_manufacturer

        // create range image and tell the pipeline nodes about it
        auto range_image =
            std::make_shared<ContinuousRangeImage>(prev_conf->num_rows,
                                                   prev_conf->num_columns,
                                                   prev_conf->num_columns * 10,
                                                   prev_conf->sensor_is_clockwise,
                                                   prev_conf->supplement_inclination_angle_for_nan_cells); // todo
        driver->setContinuousRangeImage(range_image);                                                      // todo

        // coordinate transform node requires transform info for a particular timestamp, which it requests via
        // callback
        coordinate_transformation->setRequestTransformOdomFromSensorCallback(
            [this](int64_t stamp, TransformStatus& status) { return onRequestTransformOdomFromSensor(stamp, status); });

        // register callbacks to be notified when jobs are finished at particular stages in the pipeline
        driver->setJobFinishedCallback(
            [&](const DriverResult& result)
            {
                reset_required = checkForTimeJump(result.firing);
                if (reset_required)
                    return;

                if (pub_raw_firings.getNumSubscribers() > 0)
                    pub_raw_firings.publish(firingToPointCloud(result.firing, prev_conf->sensor_frame));
            });
        range_image_generation->setJobFinishedCallback(
            [&](const RangeImageGenerationResult& result)
            { onFinishedColumn(*range_image, result.column_index, result.column_index, RANGE_IMAGE_GENERATION); });
        ground_point_segmentation->setJobFinishedCallback(
            [&](const GroundPointSegmentationResult& result)
            { onFinishedColumn(*range_image, result.column_index, result.column_index, GROUND_POINT_SEGMENTATION); });
        point_collection->setJobFinishedCallback(
            [this, &range_image](const PointCollectionResult& result)
            {
                // get fully processed columns (this locks a mutex)
                int64_t finished_column_index_start;
                int64_t finished_column_index_end;
                range_image->getColumnsRangeToClear(result.column_index,
                                                    result.outdated_minimum_required_column_indices,
                                                    finished_column_index_start,
                                                    finished_column_index_end);

                // the acquired mutex is important as otherwise this function could be interrupted in-between by
                // another finished point collection job
                onFinishedColumn(
                    *range_image, finished_column_index_start, finished_column_index_end, CONTINUOUS_CLUSTERING);

                // advance ring buffer (this unlocks above mutex)
                range_image->advanceStartOfRingBuffer(finished_column_index_start, finished_column_index_end);

                // the points were already copied from the range image; therefore, we can publish them after
                // clearance
                for (const auto& cluster_with_stamp : result.clusters_with_stamp)
                    onFinishedCluster(*cluster_with_stamp.first, cluster_with_stamp.second);
            });

        // start all threads
        driver->start();

        // subscribe to raw sensor data
        sub_sensor_data = nh_.subscribe<ethernet_msgs::Packet>(
            "raw_data",
            1000,
            [&](const ethernet_msgs::Packet::ConstPtr& msg) { driver->enqueueJob(msg); },
            ros::VoidConstPtr(),
            ros::TransportHints().tcp().tcpNoDelay(true));

        while (ros::ok() && !reset_required)
            ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));

        // shutdown raw sensor data subscriber
        sub_sensor_data.shutdown();

        // stop all threads in all pipeline nodes if already running
        driver->stop();

        // reset members
        tf_buffer.clear();
        last_update = 0;
        reset_required = false;
    }

    Eigen::Isometry3d waitForRobotFromSensorTransform(const std::string& robot_frame, const std::string& sensor_frame)
    {
        std::optional<Eigen::Isometry3d> robot_from_sensor;
        while (!robot_from_sensor)
        {
            try
            {
                geometry_msgs::TransformStamped tf =
                    tf_buffer.lookupTransform(robot_frame, sensor_frame, ros::Time(), ros::Duration(1.));
                return tf2::transformToEigen(tf.transform);
            }
            catch (tf2::TransformException& ex)
            {
                ROS_INFO_STREAM("Waiting for Transform: '" << robot_frame << "' from '" << sensor_frame << "'");
            }
        }
        ROS_INFO_STREAM("Successfully obtained transform!");
        return *robot_from_sensor;
    }

    Eigen::Isometry3d onRequestTransformOdomFromSensor(int64_t stamp, TransformStatus& status)
    {
        geometry_msgs::TransformStamped tf_latest{}, tf_at_stamp{};
        try
        {
            ros::Time t{0, 0};
            tf_latest = tf_buffer.lookupTransform(prev_conf->odom_frame, prev_conf->sensor_frame, t);
            if (!prev_conf->wait_for_tf)
            {
                status = TRANSFORM_SUCCESS;
                return tf2::transformToEigen(tf_latest.transform);
            }
            else
            {
                t.fromNSec(stamp);
                tf_at_stamp =
                    tf_buffer.lookupTransform(prev_conf->odom_frame, prev_conf->sensor_frame, t, ros::Duration(0.005));
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

    void onFinishedCluster(const std::vector<Point>& cluster_points, int64_t stamp_cluster)
    {
        pub_clusters.publish(clusterToPointCloud(cluster_points, stamp_cluster, prev_conf->odom_frame));
    }

    void onFinishedColumn(ContinuousRangeImage& range_image,
                          int64_t from_global_column_index,
                          int64_t to_global_column_index,
                          ProcessingStage stage)
    {
        auto msg = columnToPointCloud(range_image,
                                      from_global_column_index,
                                      to_global_column_index,
                                      stage == RANGE_IMAGE_GENERATION ? prev_conf->sensor_frame : prev_conf->odom_frame,
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

    bool checkForTimeJump(const RawPoints::ConstPtr& firing)
    {
        // handle time jumps
        int64_t dt = firing->stamp - last_update;
        if (last_update != 0 && std::abs(dt) > 100000000) // 100ms
        {
            ROS_WARN_STREAM("Detected jump in time (" << dt << "). Reset continuous clustering.");
            return true;
        }
        last_update = firing->stamp;
        return false;
    }

  private:
    // init ROS specific stuff
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private;
    ros::Subscriber sub_sensor_data;
    ros::Publisher pub_raw_firings;
    ros::Publisher pub_range_image_generation;
    ros::Publisher pub_ground_point_segmentation;
    ros::Publisher pub_instance_segmentation;
    ros::Publisher pub_clusters;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    dynamic_reconfigure::Server<ContinuousClusteringConfig> reconfigure_server;
    int64_t last_update{0};
    std::optional<ContinuousClusteringConfig> prev_conf;
};

} // namespace continuous_clustering

int main(int argc, char** argv)
{
    ros::init(argc, argv, "continuous_clustering");

    continuous_clustering::RosContinuousClustering node(ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();

    return 0;
}
