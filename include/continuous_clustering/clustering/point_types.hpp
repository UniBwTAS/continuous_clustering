#ifndef CONTINUOUS_CLUSTERING_POINT_TYPES_HPP
#define CONTINUOUS_CLUSTERING_POINT_TYPES_HPP

#include <continuous_clustering/clustering/general.hpp>
#include <list>
#include <memory>
#include <set>
#include <vector>

namespace continuous_clustering
{

// todo: split this header

struct RawPoint
{
    float x{std::nanf("")};
    float y{std::nanf("")};
    float z{std::nanf("")};
    float distance{std::nanf("")};          // optional: avoids to recalculate it during range image generation
    float azimuth_angle{std::nanf("")};     // optional: avoids to recalculate it during range image generation
    float inclination_angle{std::nanf("")}; // optional: avoids to recalculate it during range image generation
    int64_t firing_index{};
    uint8_t intensity{};
    int64_t stamp{};
    int64_t globally_unique_point_index{}; // used for evaluation in order to be able to obtain corresponding GT label
};

struct RawPoints
{
    int64_t stamp;
    std::vector<RawPoint> points;

    typedef std::shared_ptr<RawPoints> Ptr;
    typedef std::shared_ptr<RawPoints const> ConstPtr;
};

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
    RangeImageIndex(uint16_t row_index, int64_t column_index, int local_column_index)
        : row_index(row_index), column_index(column_index), local_column_index(local_column_index)
    {
    }

    bool operator==(const RangeImageIndex& other) const
    {
        return row_index == other.row_index && column_index == other.column_index;
    }

    bool operator!=(const RangeImageIndex& other) const
    {
        return row_index != other.row_index || column_index != other.column_index;
    }

    bool operator<(const RangeImageIndex& other) const
    {
        return row_index < other.row_index || (row_index == other.row_index && column_index < other.column_index);
    }

  public:
    uint16_t row_index{0};
    int64_t column_index{-1};
    int local_column_index{-1};
};

struct Point
{
    // range image generation / general (todo: split raw & range img)
    Point3D xyz{std::nanf(""), std::nanf(""), std::nanf("")};
    int64_t firing_index{0};
    uint8_t intensity{0};
    float distance{std::nanf("")};
    float azimuth_angle{std::nanf("")};
    float inclination_angle{std::nanf("")};
    double continuous_azimuth_angle{std::nan("")};
    int64_t global_column_index{-1};
    int local_column_index{-1};
    int row_index{-1};
    int64_t stamp{0};               // exact time stamp
    int64_t column_stamp_middle{0}; // column time stamp (middle between min & max time stamp; only filled in top row)
    int64_t column_stamp_min{std::numeric_limits<int64_t>::max()}; // time stamp of the oldest point in this column
    int64_t column_stamp_max{0};                                    // time stamp of the latest point in this column
    int64_t globally_unique_point_index{static_cast<int64_t>(-1)};

    // ground point segmentation (incl. point filtering stage)
    uint8_t debug_label{WHITE};
    uint8_t ground_point_label{GP_UNKNOWN};
    float height_over_ground{std::nanf("")};

    // clustering
    bool is_ignored{false};  // todo: move/split this field (used already in earlier stages)
    double finished_at_continuous_azimuth_angle{0.f};
    std::list<RangeImageIndex> child_points{};
    std::set<RangeImageIndex> associated_trees{};
    RangeImageIndex tree_root_{0, -1, -1};
    uint32_t tree_num_points{0};
    uint32_t cluster_width{0};
    int64_t tree_id{0};
    int64_t id{0};
    double visited_at_continuous_azimuth_angle{-1.};
    bool belongs_to_finished_cluster{false};
    int number_of_visited_neighbors{0};
};

} // namespace continuous_clustering

#endif