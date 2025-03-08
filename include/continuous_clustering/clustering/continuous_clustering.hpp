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

    // finished cluster identification
    bool is_potential_cluster_root{false};
    double finished_at_monot_azimuth_angle{0.0};

    // point collection
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
    int64_t cur_monot_col_idx;
    Eigen::Isometry3d odom_frame_from_sensor_frame;
};

struct UnionFindJob
{
    int64_t cur_monot_col_idx;
};

struct PointCollectionJob
{
    int64_t cur_monot_col_idx;
    int64_t min_required_monot_col_idx;

    std::vector<size_t> cluster_root_idxs;
};

// Structure of Arrays for Range Image - more cache-friendly memory layout
struct RangeImageSoA
{
    // raw sensor data
    std::vector<float> x; 
    std::vector<float> y;
    std::vector<float> z;
    std::vector<uint64_t> firing_idx;
    std::vector<uint8_t> intensity;
    std::vector<float> distance;
    std::vector<float> azimuth_angle;
    std::vector<float> elevation_angle;
    std::vector<uint64_t> stamp_ns;

    // range image generation
    std::vector<uint16_t> col_idx;
    std::vector<uint16_t> row_idx;
    std::vector<double> monot_azimuth_angle;
    std::vector<int64_t> monot_col_idx;
    std::vector<uint64_t> globally_unique_point_index;

    // ground point segmentation
    std::vector<uint8_t> ground_point_label;
    std::vector<bool> is_ignored;
    std::vector<float> height_over_ground;
    std::vector<uint8_t> debug_ground_point_label;

    // clustering (union find)
    std::vector<size_t> parent_idx;  // Indices instead of pointers
    std::vector<uint16_t> rank;

    // clustering (infinite cluster detection)
    std::vector<int64_t> clust_start_monot_col_idx;
    std::vector<int64_t> clust_end_monot_col_idx;

    // finished cluster identification
    std::vector<bool> is_potential_cluster_root;
    std::vector<double> finished_at_monot_azimuth_angle;

    // point collection
    std::vector<size_t> next_idx;  // Indices instead of pointers
    std::vector<uint64_t> id;

    // debugging
    std::vector<int> number_of_visited_neighbors;
    
    // Resize all arrays
    void resize(size_t size) {
        x.resize(size, std::nanf(""));
        y.resize(size, std::nanf(""));
        z.resize(size, std::nanf(""));
        firing_idx.resize(size);
        intensity.resize(size);
        distance.resize(size, std::nanf(""));
        azimuth_angle.resize(size, std::nanf(""));
        elevation_angle.resize(size, std::nanf(""));
        stamp_ns.resize(size);
        col_idx.resize(size);
        row_idx.resize(size);
        monot_azimuth_angle.resize(size, std::nan(""));
        monot_col_idx.resize(size, -1);
        globally_unique_point_index.resize(size, static_cast<uint64_t>(-1));
        ground_point_label.resize(size);
        is_ignored.resize(size);
        height_over_ground.resize(size, std::nanf(""));
        debug_ground_point_label.resize(size, WHITE);
        parent_idx.resize(size, static_cast<size_t>(-1));  // No parent initially
        rank.resize(size);
        clust_start_monot_col_idx.resize(size, -1);
        clust_end_monot_col_idx.resize(size, -1);
        is_potential_cluster_root.resize(size);
        finished_at_monot_azimuth_angle.resize(size);
        next_idx.resize(size, static_cast<size_t>(-1));  // No next initially
        id.resize(size);
        number_of_visited_neighbors.resize(size);
    }
    
    // Initialize values for a specific index
    void initializePixel(size_t index) {
        x[index] = std::nanf("");
        y[index] = std::nanf("");
        z[index] = std::nanf("");
        firing_idx[index] = 0;
        intensity[index] = 0;
        distance[index] = std::nanf("");
        azimuth_angle[index] = std::nanf("");
        elevation_angle[index] = std::nanf("");
        stamp_ns[index] = 0;
        col_idx[index] = 0;
        row_idx[index] = 0;
        monot_azimuth_angle[index] = std::nan("");
        monot_col_idx[index] = -1;
        globally_unique_point_index[index] = static_cast<uint64_t>(-1);
        ground_point_label[index] = 0;
        is_ignored[index] = false;
        height_over_ground[index] = std::nanf("");
        debug_ground_point_label[index] = WHITE;
        parent_idx[index] = static_cast<size_t>(-1);
        rank[index] = 0;
        clust_start_monot_col_idx[index] = -1;
        clust_end_monot_col_idx[index] = -1;
        is_potential_cluster_root[index] = false;
        finished_at_monot_azimuth_angle[index] = 0.0;
        next_idx[index] = static_cast<size_t>(-1);
        id[index] = 0;
        number_of_visited_neighbors[index] = 0;
    }
    
    // Clear values for a range of columns
    void clearColumns(int64_t from_monot_col_idx, int64_t to_monot_col_idx, int num_rows, int num_columns) {
        for (int64_t c = from_monot_col_idx; c <= to_monot_col_idx; ++c) {
            int ring_buf_col_idx = static_cast<int>(c % num_columns);
            for (int r = 0; r < num_rows; ++r) {
                size_t idx = ring_buf_col_idx * num_rows + r;
                initializePixel(idx);
            }
        }
    }
    
    // Helper functions for point coordinates
    inline float lengthSquared(size_t idx1, size_t idx2) const {
        float dx = x[idx1] - x[idx2];
        float dy = y[idx1] - y[idx2];
        float dz = z[idx1] - z[idx2];
        return dx*dx + dy*dy + dz*dz;
    }
    
    // Calculate distance from a point to the sensor origin
    inline float getDistance(size_t idx, const Point3D& sensor_pos) const {
        float dx = x[idx] - sensor_pos.x;
        float dy = y[idx] - sensor_pos.y;
        float dz = z[idx] - sensor_pos.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
};

// Helper functions for index conversion
inline size_t pixelIndex(int col_idx, int row_idx, int num_rows) {
    return col_idx * num_rows + row_idx;
}

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
    void calculateFovBounds(int64_t& fov_start_monot_col_idx,
                            int16_t& fov_start_row_idx,
                            int16_t& fov_end_row_idx,
                            uint16_t cur_row_idx,
                            int64_t cur_monot_col_idx,
                            float half_angular_fov);
    inline bool findEdgesInFieldOfView(size_t pixel_idx, int64_t monot_col_idx, uint16_t row_idx, float half_angular_fov);
    inline void performUnionFindForColumn(UnionFindJob&& job);
    inline void identifyFinishedClusters(int64_t cur_monot_col_idx);
    inline void collectPointsForCusterAndPublish(PointCollectionJob&& job);
    inline void clearColumns(int64_t from_monot_col_idx, int64_t to_monot_col_idx);

  public:
    // union find
    inline void make_set(Pixel* pixel, float max_angle_diff);
    inline Pixel* find_set(Pixel* pixel);
    inline bool union_set(Pixel* pixel_a, Pixel* pixel_b);
    inline void print_set(Pixel* pixel, std::vector<Pixel>& v);

    // SoA versions of union find operations
    inline void make_set_soa(size_t pixel_idx, float max_angle_diff);
    inline size_t find_set_soa(size_t pixel_idx);
    inline bool union_set_soa(size_t pixel_a_idx, size_t pixel_b_idx);
    inline void collect_set_soa(size_t root_idx, std::vector<size_t>& indices);

  public:
    // continuous range image generation
    RangeImageSoA range_image_soa_;  // New SoA range image
    std::vector<Pixel> range_image_{0};  // Keep this for backward compatibility during transition
    int num_rows_{-1};
    int num_columns_{0};
    int num_columns_rot_{};
    float azimuth_width_per_column_{};
    int64_t ring_buf_start_monot_col_idx_{};
    int64_t ring_buf_end_monot_col_idx_{};

    // Helper methods for transition
    inline size_t getIndex(int col_idx, int row_idx) const {
        return pixelIndex(col_idx, row_idx, num_rows_);
    }

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
    std::vector<float> laser_elevation_angles_;
    std::vector<size_t> potential_cluster_root_idxs_;
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
