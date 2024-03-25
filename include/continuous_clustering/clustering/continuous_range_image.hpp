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

    inline int64_t addMinimumRequiredStartColumnIndexNonGuarded(int64_t minimum_required_start_column_index,
                                                                bool allow_before_start)
    {
        if (minimum_required_start_column_index < start_column_index)
        {
            if (allow_before_start)
                minimum_required_start_column_index = start_column_index;
            else
                throw std::runtime_error(
                    "Your minimum required start column index is before the start of the range image");
        }
        minimum_required_start_column_indices[minimum_required_start_column_index]++;
        return minimum_required_start_column_index;
    }

    inline int64_t addMinimumRequiredStartColumnIndex(int64_t minimum_required_start_column_index,
                                                      bool allow_before_start)
    {
        std::lock_guard<std::mutex> lock_guard(*minimum_required_start_column_index_mutex);
        return addMinimumRequiredStartColumnIndexNonGuarded(minimum_required_start_column_index, allow_before_start);
    }

    inline void addMinimumRequiredStartColumnIndices(const std::vector<int64_t>& column_indices)
    {
        std::lock_guard<std::mutex> lock_guard(*minimum_required_start_column_index_mutex);
        for (int64_t c : column_indices)
            addMinimumRequiredStartColumnIndexNonGuarded(c, false);
    }

    inline void removeMinimumRequiredStartColumnIndexNonGuarded(int64_t minimum_required_start_column_index)
    {
        auto it = minimum_required_start_column_indices.find(minimum_required_start_column_index);
        if (it == minimum_required_start_column_indices.end() || it->second < 1)
            throw std::runtime_error(
                "There is no minimum required start column index with this value. This is invalid.");
        it->second--;
        if (it->second == 0)
            minimum_required_start_column_indices.erase(it);
    }

    inline void removeMinimumRequiredStartColumnIndex(int64_t minimum_required_start_column_index)
    {
        std::lock_guard<std::mutex> lock_guard(*minimum_required_start_column_index_mutex);
        removeMinimumRequiredStartColumnIndexNonGuarded(minimum_required_start_column_index);
    }

    inline void clearFinishedColumns(int64_t current_column_index,
                                     const std::vector<int64_t>& outdated_minimum_required_column_indices)
    {
        std::lock_guard<std::mutex> lock_guard(*minimum_required_start_column_index_mutex);
        for (int64_t c : outdated_minimum_required_column_indices)
            removeMinimumRequiredStartColumnIndexNonGuarded(c);
        int64_t minimum_required_start_column_index = minimum_required_start_column_indices.empty() ?
                                                          current_column_index + 1 :
                                                          minimum_required_start_column_indices.begin()->first;
        if (minimum_required_start_column_index < start_column_index)
            throw std::runtime_error("This shouldn't happen, ring buffer is not allowed to increase at the front: " +
                                     std::to_string(minimum_required_start_column_index) + ", " +
                                     std::to_string(start_column_index));
        if (finished_column_callback)
            finished_column_callback(start_column_index, minimum_required_start_column_index - 1);
        clearColumns(start_column_index, minimum_required_start_column_index - 1);
        start_column_index = minimum_required_start_column_index;
    }
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
