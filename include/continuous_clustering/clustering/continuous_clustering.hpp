#ifndef CONTINUOUS_CLUSTERING_CONTINUOUS_CLUSTERING_HPP
#define CONTINUOUS_CLUSTERING_CONTINUOUS_CLUSTERING_HPP

#include <set>

#include <Eigen/Geometry>

#include <continuous_clustering/clustering/continuous_range_image.hpp>
#include <continuous_clustering/clustering/general.hpp>
#include <continuous_clustering/clustering/point_types.hpp>
#include <continuous_clustering/utils/thread_pool.hpp>

namespace continuous_clustering
{

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
};

struct Configuration
{
    GeneralConfiguration general{};
    ContinuousRangeImageConfiguration range_image{};
    ContinuousGroundSegmentationConfiguration ground_segmentation{};
    ContinuousClusteringConfiguration clustering{};
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
    std::list<uint64_t> cluster_ids;
    std::list<std::list<RangeImageIndex>> trees_per_finished_cluster;
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
    inline void associatePointToPointTree(Point& point, Point& point_other, float max_angle_diff);
    inline void associatePointTreeToPointTree(const Point& point, const Point& point_other);
    inline void traverseFieldOfView(Point& point, float max_angle_diff, int ring_buffer_start_local_column_index);
    inline void associatePointsInColumn(AssociationJob&& job);
    inline void findFinishedTreesAndAssignSameId(TreeCombinationJob&& job);
    inline void collectPointsForCusterAndPublish(PublishingJob&& job);

  public:
    ContinuousRangeImage range_image{0, 0, 0, {}};

  private:
    Configuration config_;
    bool reset_required{true};

    // continuous ground point segmentation (sgps)
    std::unique_ptr<Eigen::Isometry3d> sgps_ego_robot_frame_from_sensor_frame_;

    // continuous clustering (sc)
    float max_distance_squared{0.7 * 0.7};
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

    // performance statistics
    bool stop_statistics = false;
    std::list<size_t> num_pending_jobs;
};
} // namespace continuous_clustering

#endif
