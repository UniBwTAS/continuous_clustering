#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <streaming_clustering/StreamingClusteringConfig.h>
#include <streaming_clustering/general.h>
#include <streaming_clustering/ros_transform_synchronizer.h>
#include <streaming_clustering/sensor_input.h>
#include <streaming_clustering/thread_pool.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <velodyne_pointcloud/calibration.h>

namespace streaming_clustering
{

enum
{
    GP_UNKNOWN = WHITE,       // ground point: unknown
    GP_GROUND = GREEN,        // ground point: ground
    GP_OBSTACLE = RED,        // ground point: unknown
    GP_EGO_VEHICLE = MAGENTA, // ground point: point on ego vehicle
};

class RangeImageIndex
{
  public:
    RangeImageIndex(uint16_t row_index, int64_t column_index) : row_index(row_index), column_index(column_index)
    {
    }

    bool operator==(const RangeImageIndex& other) const
    {
        return row_index == other.row_index && column_index == other.column_index;
    }

    bool operator<(const RangeImageIndex& other) const
    {
        return row_index < other.row_index || (row_index == other.row_index && column_index < other.column_index);
    }

  public:
    int64_t column_index{0};
    uint16_t row_index{0};
};

class RangeImageIndexHash
{
  public:
    std::size_t operator()(const RangeImageIndex& k) const
    {
        return (static_cast<std::size_t>(k.row_index) << 48) ^ k.column_index;
    }
};

struct Point
{
    // range image generation / general
    Point3D xyz{std::nanf(""), std::nanf(""), std::nanf("")};
    uint64_t firing_index{0};
    uint8_t intensity{0};
    float distance{std::nanf("")};
    float azimuth_angle{std::nanf("")};
    float inclination_angle{std::nanf("")};
    double continuous_azimuth_angle{std::nan("")};
    int64_t global_column_index{-1};
    int local_column_index{-1};
    int row_index{-1};
    ros::Time stamp{0, 0};

    // ground point segmentation
    uint8_t ground_point_label{0};
    float height_over_ground{std::nanf("")};
    uint8_t debug_ground_point_label{WHITE};
    std::shared_ptr<int64_t> pointer_to_global_column_index_of_next_ground_point{};
    int local_column_index_of_left_ground_neighbor{0};
    int local_column_index_of_right_ground_neighbor{0};

    // clustering
    bool is_ignored{false};
    double finished_at_continuous_azimuth_angle{0.f};
    std::list<RangeImageIndex> child_points{};
    // std::unordered_set<RangeImageIndex, RangeImageIndexHash> associated_trees{};
    std::set<RangeImageIndex> associated_trees{};
    RangeImageIndex tree_root_{0, -1};
    uint32_t tree_num_points{0};
    uint32_t id{0};
    double visited_at_continuous_azimuth_angle{-1.};
    bool belongs_to_finished_cluster{false};
    int number_of_visited_neighbors{0};
};

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

    // ground point segmentation
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_gp_label_out;
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_dbg_gp_label_out;
    sensor_msgs::PointCloud2Iterator<int32_t> iter_dbg_c_n_left_out;
    sensor_msgs::PointCloud2Iterator<int32_t> iter_dbg_c_n_right_out;
    sensor_msgs::PointCloud2Iterator<float> iter_height_over_ground_out;

    // streaming clustering
    sensor_msgs::PointCloud2Iterator<double> iter_finished_at_azimuth_angle;
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_num_child_points;
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_tree_root_row_index;
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_tree_root_column_index;
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_number_of_visited_neighbors;
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_tree_id;
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_id;
};

struct InsertionJob
{
    RawPoints::ConstPtr firing;
    geometry_msgs::TransformStamped odom_frame_from_sensor_frame;
};

struct SegmentationJob
{
    int64_t ring_buffer_current_global_column_index;
    tf2::Transform odom_frame_from_sensor_frame;
};

struct AssociationJob
{
    int64_t ring_buffer_current_global_column_index;
};

struct TreeCombinationJob
{
    int64_t ring_buffer_current_global_column_index;

    std::list<RangeImageIndex> new_unfinished_point_trees;
    double current_minimum_continuous_azimuth_angle;
};

struct PublishingJob
{
    int64_t ring_buffer_current_global_column_index;
    int64_t ring_buffer_min_required_global_column_index;

    std::list<int64_t> cluster_ids;
    std::list<std::list<RangeImageIndex>> trees_per_finished_cluster;
};

class StreamingClustering
{

  public:
    StreamingClustering(ros::NodeHandle nh, const ros::NodeHandle& nh_private);

  private:
    // general
    void reset();

    // range image generation
    void onNewFiringsCallback(const RawPoints::ConstPtr& firing);
    void onNewFiringsWithTfCallback(const RawPoints::ConstPtr& firing,
                                    const geometry_msgs::TransformStamped& odom_from_sensor);
    void insertFiringIntoRangeImage(InsertionJob&& job);

    // ground point segmentation
    inline void performGroundPointSegmentationForColumn(SegmentationJob&& job);
    static inline Point2D to2dInAzimuthPlane(const Point3D& p)
    {
        return {p.xy().length(), p.z};
    }

    // streaming clustering
    inline void associatePointsInColumn(AssociationJob&& job);
    inline void findFinishedTreesAndAssignSameId(TreeCombinationJob&& job);
    inline void collectPointsForCusterAndPublish(PublishingJob&& job);
    inline void createPointCloud2ForCluster(sensor_msgs::PointCloud2& msg,
                                            const std::vector<Point>& cluster_points,
                                            const ros::Time& min_stamp_for_this_cluster,
                                            const ros::Time& max_stamp_for_this_cluster);
    inline void clearColumns(int64_t from_global_column_index, int64_t to_global_column_index);

    // ROS-specific
    static inline PointCloud2Iterators prepareMessageAndCreateIterators(sensor_msgs::PointCloud2& msg);
    inline void addPointToMessage(PointCloud2Iterators& container, int data_index_message, const Point& point) const;
    static inline void
    addRawPointToMessage(PointCloud2Iterators& container, int data_index_message, const RawPoint& point);
    inline void publishColumns(int64_t from_global_column_index, int64_t to_global_column_index, ros::Publisher& pub);
    inline void publishFiring(const RawPoints::ConstPtr& firing);
    void callbackReconfigure(StreamingClusteringConfig& config, uint32_t level);

  private:
    // reconfigure
    dynamic_reconfigure::Server<StreamingClusteringConfig> reconfigure_server_;
    StreamingClusteringConfig config_;
    float max_distance_{};
    float max_distance_squared_{};

    // init ROS specific stuff
    ros::Publisher pub_raw_firings;
    ros::Publisher pub_ground_point_segmentation;
    ros::Publisher pub_instance_segmentation;
    ros::Publisher pub_clusters;
    ros::Time last_update_{0, 0};
    bool first_firing_after_reset{false};
    std::string ego_robot_frame{};
    std::string sensor_frame{};
    std::string odom_frame{};
    bool wait_for_tf{true};
    RosTransformSynchronizer<RawPoints> tf_synchronizer;

    // sensor specific stuff
    std::string sensor_manufacturer;
    std::shared_ptr<SensorInput> sensor_input_;

    // range image (implemented as ring buffer)
    int ring_buffer_max_columns{0};
    std::vector<Point> range_image_{0};
    int num_columns{};
    int num_rows{};
    int64_t ring_buffer_start_global_column_index{};
    int64_t ring_buffer_end_global_column_index{};

    // streaming range image generation (srig)
    float srig_azimuth_width_per_column{};
    int64_t srig_previous_global_column_index_of_rearmost_laser{0};
    int64_t srig_previous_global_column_index_of_foremost_laser{0};
    int64_t srig_first_unfinished_global_column_index{-1};
    tf2::Vector3 srig_sensor_position{0, 0, 0};
    bool srig_reset_because_of_bad_start{false};

    // streaming ground point segmentation (sgps)
    float height_ref_to_maximum_{}, height_ref_to_ground_{};
    float length_ref_to_front_end_{}, length_ref_to_rear_end_{};
    float width_ref_to_left_mirror_{}, width_ref_to_right_mirror_{};
    float height_sensor_to_ground_{};
    Point3D sgps_sensor_position{0, 0, 0};
    std::unique_ptr<tf2::Transform> sgps_ego_robot_frame_from_sensor_frame_;
    std::vector<int64_t> sgps_previous_ground_points_;
    std::vector<std::shared_ptr<int64_t>> sgps_next_ground_points_;
    ros::Time sgps_previous_column_stamp_;

    // streaming clustering (sc)
    int64_t sc_first_unpublished_global_column_index{-1};
    std::mutex sc_first_unfinished_global_column_index_mutex;
    std::list<int64_t> sc_minimum_required_global_column_indices;
    std::list<RangeImageIndex> sc_unfinished_point_trees_;
    uint32_t sc_cluster_counter_{1};
    std::vector<float> sc_inclination_angles_between_lasers_;
    bool sc_supplement_inclination_angle_for_nan_cells{true}; // allows faster association
    bool sc_use_last_point_for_cluster_stamp{false};

    // multi-threading
    ThreadPool<InsertionJob> insertion_thread_pool{"I"};
    ThreadPool<SegmentationJob> segmentation_thread_pool{"S"};
    ThreadPool<AssociationJob> association_thread_pool{"A"};
    ThreadPool<TreeCombinationJob> tree_combination_thread_pool{"C"};
    ThreadPool<PublishingJob> publishing_thread_pool{"P"};

    // performance statistics
    bool stop_statistics = false;
    std::list<size_t> num_pending_jobs;
};
} // namespace streaming_clustering
