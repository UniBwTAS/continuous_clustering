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
                         bool sensor_is_rotating_clockwise = true,
                         bool interpolate_inclination_angles_for_nan_cells = true)
        : num_rows(num_rows),
          num_columns_per_rotation(num_columns_per_rotation),
          max_columns_ring_buffer(max_columns_ring_buffer),
          azimuth_range_of_single_column(static_cast<float>((2 * M_PI)) / static_cast<float>(num_columns_per_rotation)),
          sensor_is_rotating_clockwise(sensor_is_rotating_clockwise),
          interpolate_inclination_angles_for_nan_cells(interpolate_inclination_angles_for_nan_cells),
          minimum_required_start_column_index_mutex(std::make_unique<std::mutex>())
    {
        range_image.resize(max_columns_ring_buffer * num_rows, Point());

        // used to interpolate missing inclination angles in NaN cells
        inclination_angles_between_lasers.resize(num_rows, std::nanf(""));
    }

    void
    insertFiring(RawPoints::ConstPtr& firing, int64_t& first_complete_column_index, int64_t& last_complete_column_index)
    {
        // keep track of the global column indices of the foremost and rearmost laser in this firing (w.r.t. rotation
        // direction of lidar sensor)
        int64_t column_index_of_foremost_laser = -1;
        int64_t column_index_of_rearmost_laser = -1;

        // rotation index
        int64_t previous_rotation_index_of_rearmost_laser =
            previous_column_index_of_rearmost_laser / num_columns_per_rotation;

        // the first complete column index is the one where the rearmost laser was in the previous firing
        first_complete_column_index = previous_column_index_of_rearmost_laser;
        last_complete_column_index = first_complete_column_index - 1; // no new completed column so far

        // calculate constant used below
        int columns_of_half_rotation = num_columns_per_rotation / 2;

        // ensure that there is the same number of points in the firing as number of rows in the continuous range image
        assert(firing->points.size() == num_rows);

        // process firing from top to bottom and insert each point into range image
        for (int row_index = 0; row_index < firing->points.size(); row_index++)
        {
            // obtain point
            const RawPoint& p = firing->points[row_index];

            // ignore NaN points
            if (std::isnan(p.x))
                continue;

            float azimuth_angle = std::atan2(static_cast<float>(p.y), static_cast<float>(p.x));

            // calculate azimuth angle which starts at negative X-axis with 0 and increases with ongoing lidar rotation
            // to 2 pi, which is more intuitive and important for fast array index calculation (atan2 goes from -pi -> 0
            // -> pi)
            float increasing_azimuth_angle = sensor_is_rotating_clockwise ? -azimuth_angle + static_cast<float>(M_PI) :
                                                                            azimuth_angle + static_cast<float>(M_PI);

            // calculate global column index based on the rotation index of the rearmost laser in the previous firing
            int column_index_within_rotation =
                static_cast<int>(increasing_azimuth_angle / azimuth_range_of_single_column);
            int64_t rotation_index = previous_rotation_index_of_rearmost_laser;
            int64_t column_index = rotation_index * num_columns_per_rotation + column_index_within_rotation;

            // if firing intersects negative x-axis w.r.t. previous firing we have to correct rotation_index and
            // column_index
            int column_index_within_rotation_of_previous_rearmost_laser =
                static_cast<int>(previous_column_index_of_rearmost_laser % num_columns_per_rotation);
            int column_diff = column_index_within_rotation - column_index_within_rotation_of_previous_rearmost_laser;
            if (column_diff < -columns_of_half_rotation)
            {
                rotation_index++;
                column_index += num_columns_per_rotation; // add one rotation
            }
            else if (previous_column_index_of_rearmost_laser > 0 && column_diff > columns_of_half_rotation)
            {
                // In very rare cases this can happen because the minimum azimuth of rearmost laser can be smaller than
                // that of previous firing.
                // This gets only tricky when previous_column_index_of_rearmost_laser % num_columns == 0 and
                // azimuth of rearmost laser in current firing is smaller than previous one.
                // So we have to subtract one rotation.
                rotation_index--;
                column_index -= num_columns_per_rotation; // subtract one rotation
            }

            // get point
            int local_column_index = toLocalColumnIndex(column_index);
            Point* point = &getPoint(row_index, local_column_index);

            // calculate continuous azimuth angle (even if we move it to the next cell, this value remains the same)
            double continuous_azimuth_angle =
                (2 * M_PI) * static_cast<double>(rotation_index) + increasing_azimuth_angle;

            // in case this cell is already occupied, try next column
            auto distance = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (!std::isnan(point->distance) && !std::isnan(distance))
            {
                int next_local_column_index = local_column_index + 1;
                if (next_local_column_index >= max_columns_ring_buffer)
                    next_local_column_index -= max_columns_ring_buffer;
                Point* next_point = &getPoint(row_index, next_local_column_index);
                if (std::isnan(next_point->distance))
                {
                    point = next_point;
                    local_column_index = next_local_column_index;
                    column_index++;
                }
            }

            // avoid that a valid cell (non-nan) is overwritten by a nan or more distant value
            if ((!std::isnan(point->distance) && (std::isnan(distance)) || distance >= point->distance))
                continue;

            // do not insert into columns that were considered to be finished
            bool point_in_already_published_column = false;
            if (previous_column_index_of_rearmost_laser >= 0 && column_index < previous_column_index_of_rearmost_laser)
            {
                point_in_already_published_column = true;
                std::cout << "Ignore point of firing because it would be inserted into an already published column. "
                             "Wanted to insert at "
                          << column_index << ", but rearmost laser in previous firing was already at "
                          << previous_column_index_of_foremost_laser << " (row index: " << row_index << ")"
                          << std::endl;
            }

            // fill point data
            if (!point_in_already_published_column)
            {
                point->xyz.x = static_cast<float>(p.x);
                point->xyz.y = static_cast<float>(p.y);
                point->xyz.z = static_cast<float>(p.z);
                point->firing_index = p.firing_index;
                point->intensity = p.intensity;
                point->stamp = p.stamp;
                point->distance = distance;
                point->azimuth_angle = azimuth_angle;
                point->inclination_angle = std::asin(static_cast<float>(p.z) / point->distance);
                point->continuous_azimuth_angle = continuous_azimuth_angle; // omitted cells will be filled again later
                point->global_column_index = column_index;                  // omitted cells will be filled again later
                point->local_column_index = local_column_index;
                point->row_index = row_index;
                point->globally_unique_point_index = p.globally_unique_point_index;

                // keep track of min & max time stamps in range image columns
                Point* top_row_point = &getPoint(0, local_column_index);
                if (p.stamp > top_row_point->column_stamp_max)
                    top_row_point->column_stamp_max = p.stamp;
                if (p.stamp < top_row_point->column_stamp_min)
                    top_row_point->column_stamp_min = p.stamp;
            }

            // keep track of global column index of rearmost & foremost laser
            if (column_index_of_rearmost_laser < 0 || column_index < column_index_of_rearmost_laser)
                column_index_of_rearmost_laser = column_index;
            if (column_index_of_foremost_laser < 0 || column_index > column_index_of_foremost_laser)
                column_index_of_foremost_laser = column_index;
        }

        // there was no valid point in this firing do nothing
        if (column_index_of_rearmost_laser < 0)
            return;

        // if the azimuth range of the firing covers more than 180 degrees, this means that the firing intersects with
        // negative x-axis (this means that the range image was incorrectly filled -> throw exception to signal that
        // range image must be cleared)
        if ((column_index_of_foremost_laser - column_index_of_rearmost_laser) > columns_of_half_rotation)
            throw BadStartConditionException(); // TODO: automatically handle error

        // store the global column indices of the rearmost and foremost lasers for the next firing
        if (column_index_of_rearmost_laser > previous_column_index_of_rearmost_laser)
            previous_column_index_of_rearmost_laser = column_index_of_rearmost_laser;
        if (column_index_of_foremost_laser > previous_column_index_of_foremost_laser)
            previous_column_index_of_foremost_laser = column_index_of_foremost_laser;

        // initialize start of ring buffer
        if (start_column_index == -1)
        {
            start_column_index = previous_column_index_of_rearmost_laser;
            first_complete_column_index = start_column_index;
        }

        // update end of ring buffer (maximum global column index ever seen)
        if (previous_column_index_of_foremost_laser > end_column_index)
            end_column_index = previous_column_index_of_foremost_laser;

        // consider all columns before the index of the rearmost laser as completed
        last_complete_column_index = previous_column_index_of_rearmost_laser - 1;

        // fill NaN cells and calculate the column time stamps
        for (int64_t i = first_complete_column_index; i <= last_complete_column_index; i++)
            postprocessColumn(i);
    }

    inline void postprocessColumn(int64_t column_index)
    {
        int local_column_index = toLocalColumnIndex(column_index);

        float inclination_previous_laser = 0; // calculate difference between inclination angles for subsequent steps

        // iterate from bottom to top
        for (int row_index = num_rows - 1; row_index >= 0; row_index--)
        {
            // obtain point
            Point& point = getPoint(row_index, local_column_index);

            // check if there is a problem with the ring buffer, i.e. if cell was cleared in time
            ensureClearedCell(point, column_index);

            // fill local/global column index (missing for omitted / NaN cells)
            point.global_column_index = column_index;
            point.local_column_index = local_column_index;

            // interpolate inclination and azimuth angle for NaN cells
            if (std::isnan(point.distance))
            {
                // use inclination angle difference from previous columns
                if (interpolate_inclination_angles_for_nan_cells && row_index < num_rows - 1)
                {
                    Point& point_below = getPoint(row_index + 1, local_column_index);
                    point.inclination_angle =
                        point_below.inclination_angle + inclination_angles_between_lasers[row_index];
                }
                // recalculate continuous azimuth for omitted/NaN cells (for later processing steps)
                point.continuous_azimuth_angle =
                    (static_cast<double>(column_index) + 0.5) * azimuth_range_of_single_column;
            }

            // keep track of differences between the inclination angles of the lasers (for inclination angle interpol.)
            float inclination_current_laser = point.inclination_angle;
            float diff = inclination_current_laser - inclination_previous_laser;
            if (!std::isnan(diff))
                inclination_angles_between_lasers[row_index] = diff; // last value is not used
            inclination_previous_laser = inclination_current_laser;
        }

        // calculate the column's stamp
        Point& point = getPoint(0, local_column_index);
        point.column_stamp_middle = (point.column_stamp_min >> 1) + (point.column_stamp_max >> 1);
    }

    inline void ensureClearedCell(const Point& point, int64_t column_index) const
    {
        int64_t point_global_column_index_copy = point.global_column_index;
        if (point_global_column_index_copy != column_index && point_global_column_index_copy != -1)
        {
            throw std::runtime_error(
                "This column is not cleared. Probably this means the ring buffer is full or there "
                "is some other issue with clearing (not cleared at all or written after clearing): " +
                std::to_string(point_global_column_index_copy) + ", " + std::to_string(column_index) + ", " +
                std::to_string(max_columns_ring_buffer) +
                "; This typically happens when the clustering is not fast enough to handle all the firings. "
                "Consider "
                "to play the sensor data more slowly or to adjust the parameters to make the clustering faster.");
        }
    }

    void clearColumns(int64_t from_column_index, int64_t to_column_index)
    {
        if (to_column_index < from_column_index)
            return;

        for (int64_t column_index = from_column_index; column_index <= to_column_index; column_index++)
        {
            int local_column_index = toLocalColumnIndex(column_index);
            Point empty_point = Point();
            for (int row_index = 0; row_index < num_rows; row_index++)
            {
                Point& point = getPoint(row_index, local_column_index);
                point = empty_point;
            }
        }
    }

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
    void setFinishedColumnCallback(std::function<void(int64_t, int64_t)> cb)
    {
        finished_column_callback = std::move(cb);
    }

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

    // inclination angle interpolation for NaN cells
    bool interpolate_inclination_angles_for_nan_cells{true};
    std::vector<float> inclination_angles_between_lasers;

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
