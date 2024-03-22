#ifndef CONTINUOUS_CLUSTERING_CONTINUOUS_RANGE_IMAGE_HPP
#define CONTINUOUS_CLUSTERING_CONTINUOUS_RANGE_IMAGE_HPP

#include <functional>
#include <iostream>
#include <map>
#include <mutex>

#include <continuous_clustering/clustering/point_types.hpp>

namespace continuous_clustering
{

class BadStartConditionException : public std::runtime_error
{
  public:
    explicit BadStartConditionException()
        : std::runtime_error("The first firing intersects negative x-axis. This means that the range image was filled "
                             "incorrectly. Please clear the range image and start with a firing that does not "
                             "intersect negative x-axis."){};
};

class ContinuousRangeImage
{
  public:
    ContinuousRangeImage(int num_rows,
                         int num_columns_per_rotation,
                         int max_columns_ring_buffer,
                         bool sensor_is_rotating_clockwise = true);
    void insertFiring(RawPoints::ConstPtr& firing,
                      int64_t& first_complete_column_index,
                      int64_t& last_complete_column_index);
    void clearColumns(int64_t from_column_index, int64_t to_column_index);

    int64_t addMinimumRequiredStartColumnIndex(int64_t minimum_required_start_column_index,
                                                      bool allow_before_start = false);
    void removeMinimumRequiredStartColumnIndex(int64_t minimum_required_start_column_index);
    void clearFinishedColumns(int64_t current_column_index);
    void setFinishedColumnCallback(std::function<void(int64_t, int64_t)> cb);

    inline int toLocalColumnIndex(int64_t column_index) const
    {
        return static_cast<int>(column_index % max_columns_ring_buffer);
    }

    inline Point& getPoint(int row_index, int local_column_index)
    {
        // int flattened_index = local_column_index * num_rows + row_index;
        // if (flattened_index < 0 || flattened_index >= range_image.size())
        //     throw std::runtime_error("Index Out Of Bounds: row_index: " + std::to_string(row_index) +
        //                              ", column_index: " + std::to_string(local_column_index));
        return range_image[local_column_index * num_rows + row_index]; // column major order
    }

    inline const Point& getPoint(int row_index, int local_column_index) const
    {
        return range_image[local_column_index * num_rows + row_index]; // column major order
    }

    inline Point& getPointGlobal(int row_index, int column_index)
    {
        return range_image[toLocalColumnIndex(column_index) * num_rows + row_index]; // column major order
    }

  public:
    // range image
    std::vector<Point> range_image{0};
    int num_rows{-1};
    int num_columns_per_rotation{};
    int max_columns_ring_buffer{};
    float azimuth_range_of_single_column{};
    bool sensor_is_rotating_clockwise{true};

    // multi thread column clearance
    std::map<int64_t, int> minimum_required_start_column_indices{};
    std::unique_ptr<std::mutex> minimum_required_start_column_index_mutex;
    std::function<void(int64_t, int64_t)> finished_column_callback;

    // monotonically "indefinitely" increasing start and end indices of continuous range image
    int64_t start_column_index{-1}; // starts at the rearmost laser' column index of first firing
    int64_t end_column_index{-1};

    // keep track of previous global column indices of the first and last laser in a firing
    int64_t previous_column_index_of_rearmost_laser{0};
    int64_t previous_column_index_of_foremost_laser{-1};
};
} // namespace continuous_clustering

#endif
