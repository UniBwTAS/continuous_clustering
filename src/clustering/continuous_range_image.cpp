#include <cmath>
#include <iostream>

#include <continuous_clustering/clustering/continuous_range_image.hpp>

namespace continuous_clustering
{
ContinuousRangeImage::ContinuousRangeImage(int num_rows,
                                           int num_columns_per_rotation,
                                           int max_columns_ring_buffer,
                                           bool sensor_is_rotating_clockwise)
    : num_rows(num_rows),
      num_columns_per_rotation(num_columns_per_rotation),
      max_columns_ring_buffer(max_columns_ring_buffer),
      azimuth_range_of_single_column(static_cast<float>((2 * M_PI)) / static_cast<float>(num_columns_per_rotation)),
      sensor_is_rotating_clockwise(sensor_is_rotating_clockwise),
      minimum_required_start_column_index_mutex(std::make_unique<std::mutex>())
{
    range_image.resize(max_columns_ring_buffer * num_rows, Point());
}

void ContinuousRangeImage::insertFiring(RawPoints::ConstPtr& firing,
                                        int64_t& first_complete_column_index,
                                        int64_t& last_complete_column_index)
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

    // process firing from top to bottom and insert each point into range image
    for (int row_index = 0; row_index < firing->points.size(); row_index++)
    {
        // obtain point
        const RawPoint& p = firing->points[row_index];

        if (std::isnan(p.x))
            continue;

        // calculate azimuth angle
        float azimuth_angle = std::atan2(static_cast<float>(p.y), static_cast<float>(p.x));

        // calculate azimuth angle which starts at negative X-axis with 0 and increases with ongoing lidar rotation
        // to 2 pi, which is more intuitive and important for fast array index calculation (atan2 goes from -pi -> 0 ->
        // pi)
        float increasing_azimuth_angle = sensor_is_rotating_clockwise ? -azimuth_angle + static_cast<float>(M_PI) :
                                                                        azimuth_angle + static_cast<float>(M_PI);

        // calculate global column index based on the rotation index of the rearmost laser in the previous firing
        int column_index_within_rotation = static_cast<int>(increasing_azimuth_angle / azimuth_range_of_single_column);
        int64_t rotation_index = previous_rotation_index_of_rearmost_laser;
        int64_t column_index = rotation_index * num_columns_per_rotation + column_index_within_rotation;

        // if firing intersects negative x-axis w.r.t. previous firing we have to correct rotation_index and
        // column_index
        int column_index_within_rotation_of_previous_rearmost_laser =
            static_cast<int>(previous_column_index_of_rearmost_laser % num_columns_per_rotation);
        int column_diff = column_index_within_rotation - column_index_within_rotation_of_previous_rearmost_laser;
        int columns_of_half_rotation = num_columns_per_rotation / 2;
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
        double continuous_azimuth_angle = (2 * M_PI) * static_cast<double>(rotation_index) + increasing_azimuth_angle;

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
                      << previous_column_index_of_foremost_laser << " (row index: " << row_index << ")" << std::endl;
            // std::cout << "DBG: " << local_column_index << ", " << rotation_index << ", "
            //           << previous_rotation_index_of_rearmost_laser << ", " << column_index_within_rotation << ", "
            //           << column_index_within_rotation_of_previous_rearmost_laser << std::endl;
            // throw std::runtime_error("Check if this still occurs after refactoring2");
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
    // negative x-axis (this means that the range image was incorrectly filled -> throw exception to signal that range
    // image must be cleared)
    int columns_of_half_rotation = num_columns_per_rotation / 2;
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
}

void ContinuousRangeImage::clearColumns(int64_t from_column_index, int64_t to_column_index)
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

int64_t ContinuousRangeImage::addMinimumRequiredStartColumnIndex(int64_t minimum_required_start_column_index,
                                                                 bool allow_before_start)
{
    std::lock_guard<std::mutex> lock_guard(*minimum_required_start_column_index_mutex);
    if (minimum_required_start_column_index < start_column_index)
    {
        if (allow_before_start)
            minimum_required_start_column_index = start_column_index;
        else
            throw std::runtime_error("Your minimum required start column index is before the start of the range image");
    }
    // std::cout << "ADDED MINIMUM REQUIRED COLUMN INDEX2: " << minimum_required_start_column_index << ", "
    //           << allow_before_start << std::endl;
    minimum_required_start_column_indices[minimum_required_start_column_index]++;
    return minimum_required_start_column_index;
}

void ContinuousRangeImage::removeMinimumRequiredStartColumnIndex(int64_t minimum_required_start_column_index)
{
    std::lock_guard<std::mutex> lock_guard(*minimum_required_start_column_index_mutex);
    auto it = minimum_required_start_column_indices.find(minimum_required_start_column_index);
    if (it == minimum_required_start_column_indices.end() || it->second < 1)
        throw std::runtime_error("There is no minimum required start column index with this value. This is invalid.");
    // std::cout << "REMOVE: " << it->first << ", " << it->second << std::endl;
    it->second--;
    if (it->second == 0)
    {
        // std::cout << "ERASE: " << it->first << std::endl;
        minimum_required_start_column_indices.erase(it);
    }
}

void ContinuousRangeImage::clearFinishedColumns(int64_t current_column_index)
{
    std::lock_guard<std::mutex> lock_guard(*minimum_required_start_column_index_mutex);
    int64_t minimum_required_start_column_index = minimum_required_start_column_indices.empty() ?
                                                      current_column_index + 1 :
                                                      minimum_required_start_column_indices.begin()->first;
    // std::cout << "ANDI: " << minimum_required_start_column_indices.empty() << ", "
    //           << minimum_required_start_column_index << ", "
    //           << (minimum_required_start_column_indices.empty() ? -1 :
    //                                                               minimum_required_start_column_indices.begin()->second)
    //           << std::endl;
    if (minimum_required_start_column_index < start_column_index)
        throw std::runtime_error("This shouldn't happen, ring buffer is not allowed to increase at the front: " +
                                 std::to_string(minimum_required_start_column_index) + ", " +
                                 std::to_string(start_column_index));
    if (finished_column_callback)
        finished_column_callback(start_column_index, minimum_required_start_column_index - 1);
    clearColumns(start_column_index, minimum_required_start_column_index - 1);
    start_column_index = minimum_required_start_column_index;
}

void ContinuousRangeImage::setFinishedColumnCallback(std::function<void(int64_t, int64_t)> cb)
{
    finished_column_callback = std::move(cb);
}

} // namespace continuous_clustering
