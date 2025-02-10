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

struct GeneralConfiguration
{
    bool is_single_threaded{false};
};

struct ContinuousRangeImageConfiguration
{
    bool sensor_is_clockwise{true};
    int num_columns{1700}; // rows are automatically read from number of points in firing
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
    int cluster_point_trees_every_nth_column{1};
};

struct Configuration
{
    GeneralConfiguration general{};
    ContinuousRangeImageConfiguration range_image{};
    ContinuousGroundSegmentationConfiguration ground_segmentation{};
    ContinuousClusteringConfiguration clustering{};
};

struct Point
{
    // raw sensor data
    Point3D xyz{std::nanf(""), std::nanf(""), std::nanf("")};
    uint64_t firing_idx{0};
    uint8_t intensity{0};
    float distance{std::nanf("")};
    float azimuth_angle{std::nanf("")};
    float elevation_angle{std::nanf("")};
    uint64_t stamp_ns{0};

    // range image generation
    uint16_t col_idx{0};
    uint16_t row_idx{0};
    double monot_azimuth_angle{std::nan("")};
    int64_t monot_col_idx{-1};
    uint64_t globally_unique_point_index{static_cast<uint64_t>(-1)};  // only for evaluation purposes: link to original point index in dataset

    // ground point segmentation
    uint8_t ground_point_label{0};
    bool is_ignored{false}; 
    float height_over_ground{std::nanf("")};
    uint8_t debug_ground_point_label{WHITE};

    // clustering (union find)
    Point* parent{nullptr};
    uint16_t rank{0};

    // clustering (infinite cluster detection)
    int64_t clust_start_monot_col_idx{-1};
    int64_t cluster_end_monot_col_idx{-1};

    // cluster extraction
    bool is_potential_cluster_root{false};
    double finished_at_monot_azimuth_angle{0.0};
    Point* next{nullptr}; // addition to regular "union find" for print/collect
    uint64_t id{0};

    // debugging
    int number_of_visited_neighbors{0};
};

struct InsertionJob
{
    RawPoints::ConstPtr firing;
    Eigen::Isometry3d odom_frame_from_sensor_frame;
};

struct SegmentationJob
{
    int64_t ring_buffer_current_monot_col_idx;
    Eigen::Isometry3d odom_frame_from_sensor_frame;
};

struct AssociationJob
{
    int64_t ring_buffer_current_monot_col_idx;
};

struct PublishingJob
{
    int64_t ring_buffer_current_monot_col_idx;
    int64_t ring_buffer_min_required_monot_col_idx;

    std::list<Point*> cluster_roots;
};

class ContinuousClustering
{

  public:
    ContinuousClustering();

  public:
    // general
    void reset(int num_rows);
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
    inline bool checkClusteringCondition(const Point& point, const Point& point_other) const;
    inline bool traverseFieldOfView(Point& point, float max_angle_diff, int ring_buffer_first_col_idx);
    inline void associatePointsInColumn(AssociationJob&& job);
    inline void collectPointsForCusterAndPublish(PublishingJob&& job);
    inline void clearColumns(int64_t from_monot_col_idx, int64_t to_monot_col_idx);

  public: // TODO: UF
    // union find
    inline void make_set(Point* point, float max_angle_diff);
    inline Point* find_set(Point* point);
    inline void link_set(Point* point, Point* point_other);
    inline bool union_set(Point* point, Point* point_other);
    inline void print_set(Point* point, std::vector<Point>& v);

  public:
    // range image (implemented as ring buffer)
    int ring_buffer_max_columns{0};
    int num_columns_{};
    int num_rows_{-1};
    std::vector<Point> range_image_{0};
    int64_t ring_buffer_start_monot_col_idx{};
    int64_t ring_buffer_end_monot_col_idx{};

  private:
    Configuration config_;

    // continuous range image generation (srig)
    float srig_azimuth_width_per_column{};
    int64_t srig_previous_monot_col_idx_of_rearmost_laser{0};
    int64_t srig_previous_monot_col_idx_of_foremost_laser{0};
    int64_t srig_first_unfinished_monot_col_idx{-1};
    Eigen::Vector3d srig_sensor_position{0, 0, 0};
    bool reset_required{false};

    // continuous ground point segmentation (sgps)
    Point3D sgps_sensor_position{0, 0, 0};
    std::unique_ptr<Eigen::Isometry3d> sgps_ego_robot_frame_from_sensor_frame_;

    // continuous clustering (sc)
    float max_distance_squared{0.7 * 0.7};
    int64_t sc_first_unpublished_monot_col_idx{-1};
    std::list<Point*> sc_potential_cluster_roots_;
    uint64_t sc_cluster_counter_{1};
    std::vector<float> sc_inclination_angles_between_lasers_;
    std::function<void(int64_t, int64_t, bool)> finished_column_callback_;
    std::function<void(const std::vector<Point>&, uint64_t)> finished_cluster_callback_;

    // multi-threading
    ThreadPool<InsertionJob> insertion_thread_pool{"I"};
    ThreadPool<SegmentationJob> segmentation_thread_pool{"S"};
    ThreadPool<AssociationJob> association_thread_pool{"A"};
    ThreadPool<PublishingJob> publishing_thread_pool{"P"};
    bool do_sequential_execution{false};

    // performance statistics
    bool stop_statistics = false;
    std::list<size_t> num_pending_jobs;
};
} // namespace continuous_clustering

#endif
