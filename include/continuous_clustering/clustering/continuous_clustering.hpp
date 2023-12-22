#ifndef CONTINUOUS_CLUSTERING_CONTINUOUS_CLUSTERING_HPP
#define CONTINUOUS_CLUSTERING_CONTINUOUS_CLUSTERING_HPP

#include <set>

#include <Eigen/Geometry>

#include <continuous_clustering/clustering/general.hpp>
#include <continuous_clustering/clustering/point_types.hpp>
#include <continuous_clustering/utils/thread_pool.hpp>

namespace continuous_clustering
{

enum
{
    GP_UNKNOWN = WHITE,       // ground point: unknown
    GP_GROUND = GREEN,        // ground point: ground
    GP_OBSTACLE = RED,        // ground point: unknown
    GP_EGO_VEHICLE = MAGENTA, // ground point: point on ego vehicle
    GP_FOG = LIGHTGRAY,       // ground point: classified as fog
};

struct ContinuousRangeImageConfiguration
{
    bool sensor_is_clockwise{true};
    int num_columns{1700};
    bool supplement_inclination_angle_for_nan_cells{true};
};

struct ContinuousGroundSegmentationConfiguration
{
    // General
    float max_slope{0.2};
    float first_ring_as_ground_max_allowed_z_diff{0.4};
    float first_ring_as_ground_min_allowed_z_diff{-0.4};

    // General Advanced
    float last_ground_point_slope_higher_than{-0.1};
    float last_ground_point_distance_smaller_than{5.};
    float ground_because_close_to_last_certain_ground_max_z_diff{0.4};
    float ground_because_close_to_last_certain_ground_max_dist_diff{2.0};
    float obstacle_because_next_certain_obstacle_max_dist_diff{0.3};

    // Segmentation by terrain
    bool use_terrain{false};
    float terrain_max_allowed_z_diff{0.4};

    // Detection of points on the ego robot
    float height_ref_to_maximum_{}, height_ref_to_ground_{};
    float length_ref_to_front_end_{}, length_ref_to_rear_end_{};
    float width_ref_to_left_mirror_{}, width_ref_to_right_mirror_{};

    // Filter points originating from fog
    bool fog_filtering_enabled{false};
    uint8_t fog_filtering_intensity_below{2};
    float fog_filtering_distance_below{18};
    float fog_filtering_inclination_above{-0.06};

    // TODO: ego bounding box + frame!
};

struct ContinuousClusteringConfiguration
{
    float max_distance{0.7};
    int max_steps_in_row{20};
    int max_steps_in_column{20};
    bool stop_after_association_enabled{true};
    int stop_after_association_min_steps{1};
    bool ignore_points_in_chessboard_pattern{true};
    bool ignore_points_with_too_big_inclination_angle_diff{true};
    bool use_last_point_for_cluster_stamp{false};
};

struct Configuration
{
    ContinuousRangeImageConfiguration range_image{};
    ContinuousGroundSegmentationConfiguration ground_segmentation{};
    ContinuousClusteringConfiguration clustering{};
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
    uint64_t stamp{0};
    uint64_t globally_unique_point_index{static_cast<uint64_t>(-1)};

    // ground point segmentation
    uint8_t ground_point_label{0};
    float height_over_ground{std::nanf("")};
    uint8_t debug_ground_point_label{WHITE};

    // clustering
    bool is_ignored{false};
    double finished_at_continuous_azimuth_angle{0.f};
    std::list<RangeImageIndex> child_points{};
    // std::unordered_set<RangeImageIndex, RangeImageIndexHash> associated_trees{};
    std::set<RangeImageIndex> associated_trees{};
    RangeImageIndex tree_root_{0, -1};
    uint32_t tree_num_points{0};
    uint64_t id{0};
    double visited_at_continuous_azimuth_angle{-1.};
    bool belongs_to_finished_cluster{false};
    int number_of_visited_neighbors{0};
};

struct InsertionJob
{
    RawPoints::ConstPtr firing;
    Eigen::Isometry3d odom_frame_from_sensor_frame;
};

struct SegmentationJob
{
    int64_t ring_buffer_current_global_column_index;
    Eigen::Isometry3d odom_frame_from_sensor_frame;
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

    std::list<uint64_t> cluster_ids;
    std::list<std::list<RangeImageIndex>> trees_per_finished_cluster;
};

class ContinuousClustering
{

  public:
    ContinuousClustering();

  public:
    // general
    void reset(int num_rows, bool sequential_execution = false);
    void setConfiguration(const Configuration& config);
    bool resetRequired() const;

    // range image generation
    void addFiring(const RawPoints::ConstPtr& firing, const Eigen::Isometry3d& odom_from_sensor);

    // ground point segmentation
    void setTransformRobotFrameFromSensorFrame(const Eigen::Isometry3d& tf);
    bool hasTransformRobotFrameFromSensorFrame();

    // continuous clustering
    void setFinishedColumnCallback(std::function<void(int64_t, int64_t, bool)> cb);
    void setFinishedClusterCallback(std::function<void(const std::vector<Point>&, uint64_t)> cb);

    // debugging
    void recordJobQueueWorkload(size_t num_jobs_sensor_input); // debugging

  private:
    // range image generation
    void insertFiringIntoRangeImage(InsertionJob&& job);

    // ground point segmentation
    inline void performGroundPointSegmentationForColumn(SegmentationJob&& job);
    static inline Point2D to2dInAzimuthPlane(const Point3D& p)
    {
        return {p.xy().length(), p.z};
    }

    // continuous clustering
    inline void associatePointsInColumn(AssociationJob&& job);
    inline void findFinishedTreesAndAssignSameId(TreeCombinationJob&& job);
    inline void collectPointsForCusterAndPublish(PublishingJob&& job);
    inline void clearColumns(int64_t from_global_column_index, int64_t to_global_column_index);

  public:
    // range image (implemented as ring buffer)
    int ring_buffer_max_columns{0};
    int num_columns_{};
    int num_rows_{-1};
    std::vector<Point> range_image_{0};
    int64_t ring_buffer_start_global_column_index{};
    int64_t ring_buffer_end_global_column_index{};

  private:
    Configuration config_;

    // continuous range image generation (srig)
    float srig_azimuth_width_per_column{};
    int64_t srig_previous_global_column_index_of_rearmost_laser{0};
    int64_t srig_previous_global_column_index_of_foremost_laser{0};
    int64_t srig_first_unfinished_global_column_index{-1};
    Eigen::Vector3d srig_sensor_position{0, 0, 0};
    bool reset_required{false};

    // continuous ground point segmentation (sgps)
    Point3D sgps_sensor_position{0, 0, 0};
    std::unique_ptr<Eigen::Isometry3d> sgps_ego_robot_frame_from_sensor_frame_;

    // continuous clustering (sc)
    float max_distance_squared{0.7 * 0.7};
    int64_t sc_first_unpublished_global_column_index{-1};
    std::mutex sc_first_unfinished_global_column_index_mutex;
    std::list<int64_t> sc_minimum_required_global_column_indices;
    std::list<RangeImageIndex> sc_unfinished_point_trees_;
    uint64_t sc_cluster_counter_{1};
    std::vector<float> sc_inclination_angles_between_lasers_;
    std::function<void(int64_t, int64_t, bool)> finished_column_callback_;
    std::function<void(const std::vector<Point>&, uint64_t)> finished_cluster_callback_;

    // multi-threading
    ThreadPool<InsertionJob> insertion_thread_pool{"I"};
    ThreadPool<SegmentationJob> segmentation_thread_pool{"S"};
    ThreadPool<AssociationJob> association_thread_pool{"A"};
    ThreadPool<TreeCombinationJob> tree_combination_thread_pool{"C"};
    ThreadPool<PublishingJob> publishing_thread_pool{"P"};
    bool do_sequential_execution{false};

    // performance statistics
    bool stop_statistics = false;
    std::list<size_t> num_pending_jobs;
};
} // namespace continuous_clustering

#endif
