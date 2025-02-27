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
    int num_columns_rot{1700}; // rows are automatically read from number of points in firing
    bool supplement_elevation_angle_for_nan_cells{true};
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
    float fog_filtering_elevation_above{-0.06};

    // TODO: ego bounding box + frame!
};

struct ContinuousClusteringConfiguration
{
    float max_distance{0.7};
    int max_steps_in_row{20};
    int max_steps_in_column{20};
    bool stop_after_first_edge_enabled{true};
    int stop_after_first_edge_min_steps{1};
    bool ignore_pixels_in_chessboard_pattern{true};
    bool ignore_pixels_in_every_second_row{false};
    bool ignore_pixels_in_every_second_column{false};
    bool ignore_pixels_with_too_big_elevation_angle_diff{true};
    bool use_last_point_for_cluster_stamp{false};
};

struct Configuration
{
    GeneralConfiguration general{};
    ContinuousRangeImageConfiguration range_image{};
    ContinuousGroundSegmentationConfiguration ground_segmentation{};
    ContinuousClusteringConfiguration clustering{};
};

struct Pixel
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
    uint64_t globally_unique_point_index{
        static_cast<uint64_t>(-1)}; // only for evaluation purposes: link to original point index in dataset

    // ground point segmentation
    uint8_t ground_point_label{0};
    bool is_ignored{false};
    float height_over_ground{std::nanf("")};
    uint8_t debug_ground_point_label{WHITE};

    // clustering (union find)
    Pixel* parent{nullptr};
    uint16_t rank{0};

    // clustering (infinite cluster detection)
    int64_t clust_start_monot_col_idx{-1};
    int64_t clust_end_monot_col_idx{-1};

    // cluster extraction
    bool is_potential_cluster_root{false};
    double finished_at_monot_azimuth_angle{0.0};
    Pixel* next{nullptr}; // addition to regular "union find" for print/collect
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
    int64_t current_monot_col_idx;
    Eigen::Isometry3d odom_frame_from_sensor_frame;
};

struct UnionFindJob
{
    int64_t current_monot_col_idx;
};

struct FinishedClusterExtractionJob
{
    int64_t current_monot_col_idx;
    double min_monot_azimuth_angle_in_col;
};

struct PointCollectionJob
{
    int64_t current_monot_col_idx;
    int64_t min_required_monot_col_idx;

    std::vector<Pixel*> cluster_roots;
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
    void setFinishedClusterCallback(std::function<void(const std::vector<Pixel>&, uint64_t)> cb);

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
    inline bool checkClusteringCondition(const Pixel& pixel_a, const Pixel& pixel_b) const;
    inline bool findEdgesInFieldOfView(Pixel& pixel, float max_angle_diff, int ring_buf_first_col_idx);
    inline void performUnionFindForColumn(UnionFindJob&& job);
    inline void extractFinishedClusters(FinishedClusterExtractionJob&& job);
    inline void collectPointsForCusterAndPublish(PointCollectionJob&& job);
    inline void clearColumns(int64_t from_monot_col_idx, int64_t to_monot_col_idx);

  public:
    // union find
    inline void make_set(Pixel* pixel, float max_angle_diff);
    inline Pixel* find_set(Pixel* pixel);
    inline bool union_set(Pixel* pixel_a, Pixel* pixel_b);
    inline void print_set(Pixel* pixel, std::vector<Pixel>& v);

  public:
    // continuous range image generation
    std::vector<Pixel> range_image_{0};
    int num_rows_{-1};
    int num_columns_{0};
    int num_columns_rot_{};
    float azimuth_width_per_column_{};
    int64_t ring_buf_start_monot_col_idx_{};
    int64_t ring_buf_end_monot_col_idx_{};

  private:
    Configuration config_;

    // continuous range image generation
    int64_t min_incomlete_monot_col_idx_{0};
    int64_t min_unfinished_monot_col_idx_{-1};
    Eigen::Vector3d sensor_position_{0, 0, 0};
    bool reset_required_{false};

    // continuous ground point segmentation
    Point3D sensor_position_point_{0, 0, 0};
    std::unique_ptr<Eigen::Isometry3d> ego_robot_frame_from_sensor_frame_;

    // clustering (union find & cluster extraction & point collection)
    float max_distance_squared_{0.7 * 0.7};
    std::vector<float> elevation_angles_between_lasers_;
    std::vector<Pixel*> potential_cluster_roots_;
    std::function<void(int64_t, int64_t, bool)> finished_column_callback_;
    std::function<void(const std::vector<Pixel>&, uint64_t)> finished_cluster_callback_;

    // multi-threading
    ThreadPool<InsertionJob> range_image_thread_pool_{"R"};
    ThreadPool<SegmentationJob> ground_segmentation_thread_pool_{"S"};
    ThreadPool<UnionFindJob> union_find_thread_pool_{"U"};
    ThreadPool<PointCollectionJob> point_collection_thread_pool_{"C"};
    bool do_sequential_execution_{false};

    // performance statistics
    bool stop_statistics_ = false;
    std::list<size_t> num_pending_jobs_;
};
} // namespace continuous_clustering

#endif
