#include <continuous_clustering/clustering/continuous_clustering.hpp>

#include <iostream>
#include <utility>

namespace continuous_clustering
{

ContinuousClustering::ContinuousClustering() = default;

void ContinuousClustering::reset(int num_rows)
{
    // shutdown workers
    range_image_thread_pool_.shutdown();
    ground_segmentation_thread_pool_.shutdown();
    union_find_thread_pool_.shutdown();
    point_collection_thread_pool_.shutdown();

    // init/reset range image (implemented as ring buffer)
    num_columns_rot_ = config_.range_image.num_columns_rot;
    azimuth_width_per_column_ = static_cast<float>((2 * M_PI)) / static_cast<float>(num_columns_rot_);
    range_image_.resize(num_columns_rot_ * 10, num_rows);
    range_image_.clearColumns(0, range_image_.width - 1);
    ring_buf_start_monot_col_idx_ = -1; // does not start at zero but at the minimum laser of first firing
    ring_buf_end_monot_col_idx_ = -1;

    // reset members for continuous range image generation
    min_incomlete_monot_col_idx_ = 0;
    reset_required_ = false;

    // reset members for continuous ground point segmentation (sgps)
    ego_robot_frame_from_sensor_frame_.reset();

    // reset members for continuous clustering
    min_unfinished_monot_col_idx_ = -1;
    laser_elevation_angles_.resize(num_rows, std::nanf(""));
    potential_cluster_root_idxs_.clear();

    // re-initialize workers
    int num_treads = config_.general.is_single_threaded ? 0 : 1;
    int num_treads_pub = config_.general.is_single_threaded ? 0 : 1;
    range_image_thread_pool_.init(
        [this](InsertionJob&& job) { insertFiringIntoRangeImage(std::forward<InsertionJob>(job)); }, num_treads);
    ground_segmentation_thread_pool_.init(
        [this](SegmentationJob&& job) { performGroundPointSegmentationForColumn(std::forward<SegmentationJob>(job)); },
        num_treads);
    union_find_thread_pool_.init(
        [this](UnionFindJob&& job) { performUnionFindForColumn(std::forward<UnionFindJob>(job)); }, num_treads);
    point_collection_thread_pool_.init([this](PointCollectionJob&& job)
                                       { collectPointsForCusterAndPublish(std::forward<PointCollectionJob>(job)); },
                                       num_treads_pub);
}

void ContinuousClustering::setConfiguration(const Configuration& config)
{
    // some parameter changes need a hard reset
    if (config_.general.is_single_threaded != config.general.is_single_threaded)
        reset_required_ = true;
    if (config_.range_image.sensor_is_clockwise != config.range_image.sensor_is_clockwise)
        reset_required_ = true;
    if (config_.range_image.num_columns_rot != config.range_image.num_columns_rot)
        reset_required_ = true;

    // save new config
    config_ = config;

    // recalculate some values
    max_distance_squared_ = config_.clustering.max_distance * config_.clustering.max_distance;
}

bool ContinuousClustering::resetRequired() const
{
    return reset_required_;
}

void ContinuousClustering::addFiring(const RawPoints::ConstPtr& firing, const Eigen::Isometry3d& odom_from_sensor)
{
    if (range_image_.height != firing->points.size())
        throw std::runtime_error("The number of points in a firing has changed. This is probably a bug!");
    range_image_thread_pool_.enqueue({firing, odom_from_sensor});
}

void ContinuousClustering::setFinishedColumnCallback(std::function<void(int64_t, int64_t, bool)> cb)
{
    finished_column_callback_ = std::move(cb);
}

void ContinuousClustering::setFinishedClusterCallback(std::function<void(const std::vector<uint64_t>&, uint64_t)> cb)
{
    finished_cluster_callback_ = std::move(cb);
}

void ContinuousClustering::insertFiringIntoRangeImage(InsertionJob&& job)
{
    // save sensor position in odom frame
    sensor_position_ = job.odom_frame_from_sensor_frame.translation();

    // sensor position
    sensor_position_point_.x = static_cast<float>(sensor_position_.x());
    sensor_position_point_.y = static_cast<float>(sensor_position_.y());
    sensor_position_point_.z = static_cast<float>(sensor_position_.z());

    // keep track of the global column indices of the foremost and rearmost laser (w.r.t. azimuth angle clockwise =
    // rotation direction of lidar sensor) in this firing
    int64_t monot_col_idx_of_foremost_laser = -1;
    int64_t monot_col_idx_of_rearmost_laser = -1;

    // rotation index (not the same for all points if firing intersects with negative x-axis)
    int64_t approximate_rotation_index = min_incomlete_monot_col_idx_ / num_columns_rot_;

    // for rotation index correction (large jump means that the firing intersects negative x-axis)
    int cols_of_half_rotation = num_columns_rot_ / 2;

    // process firing from top to bottom
    for (int row_idx = 0; row_idx < job.firing->points.size(); row_idx++)
    {
        // obtain point
        const RawPoint& raw_point = job.firing->points[row_idx];
        Eigen::Vector3d p(raw_point.x, raw_point.y, raw_point.z);

        if (std::isnan(p.x()))
            continue;

        // calculate azimuth angle
        float azimuth_angle = std::atan2(static_cast<float>(p.y()), static_cast<float>(p.x()));

        // calculate azimuth angle which starts at negative X-axis with 0 and increases with ongoing lidar rotation
        // to 2*pi, which is more intuitive and important for fast array index calculation
        float increasing_azimuth_angle = config_.range_image.sensor_is_clockwise ?
                                             -azimuth_angle + static_cast<float>(M_PI) :
                                             azimuth_angle + static_cast<float>(M_PI);

        // calculate column index within rotation
        int column_index_within_rotation = static_cast<int>(increasing_azimuth_angle / azimuth_width_per_column_);

        // correct rotation index in case the firing intersects negative x-axis with w.r.t. previous firing
        int64_t rotation_index = approximate_rotation_index;

        // this is done by detecting jumps of more than a half rotation
        int ref_column_index_within_rotation = static_cast<int>(min_incomlete_monot_col_idx_ % num_columns_rot_);
        int column_diff = column_index_within_rotation - ref_column_index_within_rotation;
        if (column_diff < -cols_of_half_rotation)
        {
            // negative jump of more than a half rotation -> this point belongs already to the next rotation
            rotation_index += 1;
        }
        else if (min_incomlete_monot_col_idx_ > 0 && column_diff > cols_of_half_rotation)
        {
            // positive jump of more than a half rotation -> this point belongs to the previous rotation:
            // In very rare cases this can happen because the minimum azimuth of rearmost laser can be smaller than
            // that of previous firing. Logically this should be impossible as the sensor never rotates backwards.
            // However, for some reason it still happens. Maybe due to rounding/numerical errors or due to ego motion
            // correction?
            rotation_index -= 1;
        }

        // monotonic column index
        int64_t monot_col_idx = rotation_index * num_columns_rot_ + column_index_within_rotation;

        // calculate regular column index
        uint16_t col_idx = range_image_.fromMonotColIdx(monot_col_idx);

        // get current pixel index
        uint64_t pixel_idx = range_image_.getIndex(col_idx, row_idx);

        // calculate continuous azimuth angle (even if we move it to the next cell, this value remains the same)
        double monot_azimuth_angle = (2 * M_PI) * static_cast<double>(rotation_index) + increasing_azimuth_angle;

        // in case this cell is already occupied, try next column
        auto distance = static_cast<float>(p.norm());
        if (!std::isnan(range_image_.distance[pixel_idx]) && !std::isnan(distance))
        {
            int next_col_idx = col_idx + 1;
            if (next_col_idx >= range_image_.width)
                next_col_idx -= range_image_.width;
            uint64_t next_pixel_idx = range_image_.getIndex(next_col_idx, row_idx);
            if (std::isnan(range_image_.distance[next_pixel_idx]))
            {
                pixel_idx = next_pixel_idx;
                col_idx = next_col_idx;
                monot_col_idx++;
            }
        }

        // avoid that a valid cell (non-nan) is overwritten by a nan or more distant value
        if (!std::isnan(range_image_.distance[pixel_idx]) &&
            (std::isnan(distance) || distance >= range_image_.distance[pixel_idx]))
            continue;

        // do not insert into cols that were passed to the next processing step
        bool laser_too_far_behind = false;
        if (min_incomlete_monot_col_idx_ >= 0 && monot_col_idx < min_incomlete_monot_col_idx_)
        {
            /*ROS_WARN_STREAM("Ignore point of firing because it would be inserted into an already published column. "
                            "Wanted to insert at "
                            << monot_col_idx << ", but first unfinished global column index is already at "
                            << min_incomlete_monot_col_idx_ << " (row index: " << row_index << ")");*/

            laser_too_far_behind = true;
        }

        // fill pixel data
        if (!laser_too_far_behind)
        {
            // transform point into odom
            Eigen::Vector3d p_odom = job.odom_frame_from_sensor_frame * p;

            // fill range image
            range_image_.x[pixel_idx] = static_cast<float>(p_odom.x());
            range_image_.y[pixel_idx] = static_cast<float>(p_odom.y());
            range_image_.z[pixel_idx] = static_cast<float>(p_odom.z());
            range_image_.firing_idx[pixel_idx] = raw_point.firing_index;
            range_image_.intensity[pixel_idx] = raw_point.intensity;
            range_image_.stamp_ns[pixel_idx] = raw_point.stamp;
            range_image_.distance[pixel_idx] = distance;
            range_image_.azimuth_angle[pixel_idx] = azimuth_angle;
            range_image_.elevation_angle[pixel_idx] = std::asin(static_cast<float>(p.z()) / distance);
            range_image_.monot_azimuth_angle[pixel_idx] = monot_azimuth_angle;
            range_image_.col_idx[pixel_idx] = col_idx;
            range_image_.row_idx[pixel_idx] = row_idx;
            range_image_.monot_col_idx[pixel_idx] = monot_col_idx;
            range_image_.globally_unique_point_index[pixel_idx] = raw_point.globally_unique_point_index;
        }

        // keep track of global column index of rearmost & foremost laser
        if (monot_col_idx_of_rearmost_laser < 0 || monot_col_idx < monot_col_idx_of_rearmost_laser)
            monot_col_idx_of_rearmost_laser = monot_col_idx;
        if (monot_col_idx_of_foremost_laser < 0 || monot_col_idx > monot_col_idx_of_foremost_laser)
            monot_col_idx_of_foremost_laser = monot_col_idx;
    }

    // if there were no valid points in this firing, interrupt here
    if (monot_col_idx_of_rearmost_laser < 0)
        return;

    // if the azimuth range of the firing covers more than 180 degrees, this means that the firing is
    // intersected with negative x-axis (this means that the range image was incorrectly filled -> reset)
if ((monot_col_idx_of_foremost_laser - monot_col_idx_of_rearmost_laser) > cols_of_half_rotation)
    {
        std::cout << "Very first firing after reset intersects with negative x-axis: " +
                         std::to_string(monot_col_idx_of_rearmost_laser) + ", " +
                         std::to_string(monot_col_idx_of_foremost_laser) + ", " +
                         std::to_string(min_incomlete_monot_col_idx_) +
                         ". This is invalid. Reset continuous clustering on next message.";
        reset_required_ = true;
        return;
    }

    // initialize start of ring buffer
    if (ring_buf_start_monot_col_idx_ == -1)
    {
        ring_buf_start_monot_col_idx_ = monot_col_idx_of_rearmost_laser;
        min_unfinished_monot_col_idx_ = monot_col_idx_of_rearmost_laser;
        min_incomlete_monot_col_idx_ = monot_col_idx_of_rearmost_laser;
    }

    // update end of ring buffer (maximum global column index ever seen)
    if (monot_col_idx_of_foremost_laser > ring_buf_end_monot_col_idx_)
        ring_buf_end_monot_col_idx_ = monot_col_idx_of_foremost_laser;

    // iterate over finished but unfinished cols and publish them
    while (min_incomlete_monot_col_idx_ < monot_col_idx_of_rearmost_laser)
        ground_segmentation_thread_pool_.enqueue(
            {static_cast<uint64_t>(min_incomlete_monot_col_idx_++), job.odom_frame_from_sensor_frame});
}

void ContinuousClustering::performGroundPointSegmentationForColumn(SegmentationJob&& job)
{
    int col_idx = range_image_.fromMonotColIdx(job.cur_monot_col_idx);

    if (!ego_robot_frame_from_sensor_frame_)
        throw std::runtime_error("Transform robot frame from sensor frame was not set yet!");
    Eigen::Isometry3d ego_robot_frame_from_odom_frame =
        *ego_robot_frame_from_sensor_frame_ * job.odom_frame_from_sensor_frame.inverse();
    float height_sensor_to_ground = -static_cast<float>(ego_robot_frame_from_sensor_frame_->translation().z()) +
                                    config_.ground_segmentation.height_ref_to_ground_;

    // iterate rows from bottom to top and find ground points
    bool first_obstacle_detected = false;
    bool first_point_found = false;

    // Use separate variables for last ground position
    float last_ground_x = 0;
    float last_ground_y = 0;
    float last_ground_z = height_sensor_to_ground;

    float previous_x = 0;
    float previous_y = 0;
    float previous_z = 0;
    uint8_t previous_label;

    for (int row_index = range_image_.height - 1; row_index >= 0; row_index--)
    {
        // get pixel index
        uint64_t index = range_image_.getIndex(col_idx, row_index);

        // check if there is a problem with the ring buffer
        int64_t pixel_monot_col_idx_copy = range_image_.monot_col_idx[index];
        if (pixel_monot_col_idx_copy != job.cur_monot_col_idx && pixel_monot_col_idx_copy != -1)
        {
            stop_statistics_ = true;
            /*std::string filename = std::tmpnam(nullptr);
            std::cout << "JOB QUEUES (INSERT, SEGMENT, ASSOC, PUB): "
                      << range_image_thread_pool_.getNumberOfUnprocessedJobs() << ", "
                      << ground_segmentation_thread_pool_.getNumberOfUnprocessedJobs() << ", "
                      << union_find_thread_pool_.getNumberOfUnprocessedJobs() << ", "
                      << point_collection_thread_pool_.getNumberOfUnprocessedJobs() << std::endl;
            std::cout << "Writing statistics to: " << filename << std::endl;
            std::ofstream out(filename);
            for (auto n : num_pending_jobs_)
                out << n << ", ";
            out.close();*/
            throw std::runtime_error(
                "This column is not cleared. Probably this means the ring buffer is full or there "
                "is some other issue with clearing (not cleared at all or written after clearing): " +
                std::to_string(pixel_monot_col_idx_copy) + ", " + std::to_string(job.cur_monot_col_idx) + ", " +
                std::to_string(range_image_.width) +
                "; This typically happens when the clustering is not fast enough to handle all the firings. Consider "
                "to play the sensor data more slowly or to adjust the parameters to make the clustering faster.");
        }

        // refill local/global column index because it was not filled for omitted cells
        range_image_.monot_col_idx[index] = job.cur_monot_col_idx;
        range_image_.col_idx[index] = range_image_.fromMonotColIdx(job.cur_monot_col_idx);

        // keep track of the elevation angles of the lasers (for later processing steps)
        float elevation_current_laser = range_image_.elevation_angle[index];
        if (!std::isnan(elevation_current_laser))
            laser_elevation_angles_[row_index] = elevation_current_laser;

        // skip NaN's
        if (std::isnan(range_image_.distance[index]))
        {
            // fill with data if NaN
            range_image_.elevation_angle[index] = laser_elevation_angles_[row_index];
            // recalculate continuous azimuth for omitted/NaN cells (for later processing steps)
            range_image_.monot_azimuth_angle[index] =
                (static_cast<double>(job.cur_monot_col_idx) + 0.5) * azimuth_width_per_column_;
            continue;
        }

        // skip pixels which seem to be fog
        if (config_.ground_segmentation.fog_filtering_enabled &&
            range_image_.intensity[index] < config_.ground_segmentation.fog_filtering_intensity_below &&
            range_image_.distance[index] < config_.ground_segmentation.fog_filtering_distance_below &&
            range_image_.elevation_angle[index] > config_.ground_segmentation.fog_filtering_elevation_above)
        {
            range_image_.ground_point_label[index] = GP_FOG;
            range_image_.debug_ground_point_label[index] = LIGHTGRAY;
            continue;
        }

        // Calculate position with respect to sensor using separate x, y, z components
        float current_x_wrt_sensor = range_image_.x[index] - sensor_position_point_.x;
        float current_y_wrt_sensor = range_image_.y[index] - sensor_position_point_.y;
        float current_z_wrt_sensor = range_image_.z[index] - sensor_position_point_.z;

        // special handling for points on ego vehicle surface
        Eigen::Vector3d current_position_in_ego_robot_frame =
            ego_robot_frame_from_odom_frame *
            Eigen::Vector3d(range_image_.x[index], range_image_.y[index], range_image_.z[index]);
        const auto& c = config_.ground_segmentation;
        if (current_position_in_ego_robot_frame.x() < c.length_ref_to_front_end_ &&
            current_position_in_ego_robot_frame.x() > c.length_ref_to_rear_end_ &&
            current_position_in_ego_robot_frame.y() < c.width_ref_to_left_mirror_ &&
            current_position_in_ego_robot_frame.y() > c.width_ref_to_right_mirror_ &&
            current_position_in_ego_robot_frame.z() < c.height_ref_to_maximum_ &&
            current_position_in_ego_robot_frame.z() > c.height_ref_to_ground_)
        {
            range_image_.ground_point_label[index] = GP_EGO_VEHICLE;
            range_image_.debug_ground_point_label[index] = VIOLET;
            continue;
        }

        // special handling first point outside the ego vehicle box
        if (!first_point_found)
        {
            // now we found the first point outside the ego vehicle box
            first_point_found = true;
            float height_over_predicted_ground = current_z_wrt_sensor - height_sensor_to_ground;
            if (height_over_predicted_ground > c.first_ring_as_ground_min_allowed_z_diff &&
                height_over_predicted_ground < c.first_ring_as_ground_max_allowed_z_diff)
            {
                range_image_.ground_point_label[index] = GP_GROUND;
                range_image_.debug_ground_point_label[index] = GRAY;

                last_ground_x = current_x_wrt_sensor;
                last_ground_y = current_y_wrt_sensor;
                last_ground_z = current_z_wrt_sensor;
                first_obstacle_detected = false;
            }
            else
            {
                range_image_.ground_point_label[index] = GP_OBSTACLE;
                range_image_.debug_ground_point_label[index] = ORANGE;

                first_obstacle_detected = true;
            }
            previous_x = current_x_wrt_sensor;
            previous_y = current_y_wrt_sensor;
            previous_z = current_z_wrt_sensor;
            previous_label = range_image_.debug_ground_point_label[index];
            continue;
        }

        // Calculate 2D positions in azimuth plane for slope calculation
        // To2dInAzimuthPlane function converted to work with separate x, y, z
        float xy_length_current =
            std::sqrt(current_x_wrt_sensor * current_x_wrt_sensor + current_y_wrt_sensor * current_y_wrt_sensor);
        float xy_length_previous = std::sqrt(previous_x * previous_x + previous_y * previous_y);
        float xy_length_last_ground = std::sqrt(last_ground_x * last_ground_x + last_ground_y * last_ground_y);

        // calculate the slope w.r.t previous point
        float current_2d_x = xy_length_current;
        float current_2d_y = current_z_wrt_sensor;
        float previous_2d_x = xy_length_previous;
        float previous_2d_y = previous_z;

        float prev_to_current_x = current_2d_x - previous_2d_x;
        float prev_to_current_y = current_2d_y - previous_2d_y;

        float slope_to_prev = prev_to_current_y / prev_to_current_x;
        bool is_flat_wrt_prev = std::abs(slope_to_prev) < c.max_slope && prev_to_current_x > 0;

        // calculate slope w.r.t. last seen (quite certain) ground point
        float last_ground_2d_x = xy_length_last_ground;
        float last_ground_2d_y = last_ground_z;

        float last_ground_to_current_x = current_2d_x - last_ground_2d_x;
        float last_ground_to_current_y = current_2d_y - last_ground_2d_y;

        float slope_to_last_ground = last_ground_to_current_y / last_ground_to_current_x;
        bool is_flat_wrt_last_ground = std::abs(slope_to_last_ground) < c.max_slope && last_ground_to_current_x > 0;

        // quite certain ground points
        if (!first_obstacle_detected && is_flat_wrt_prev)
        {
            range_image_.ground_point_label[index] = GP_GROUND;
            range_image_.debug_ground_point_label[index] = GREEN;
        }
        else // try to find remaining ground points
        {
            if (first_obstacle_detected && is_flat_wrt_prev && is_flat_wrt_last_ground)
            {
                range_image_.ground_point_label[index] = GP_GROUND;
                range_image_.debug_ground_point_label[index] = YELLOWGREEN;
            }
            else if (std::abs(last_ground_to_current_x) < c.ground_because_close_to_last_certain_ground_max_dist_diff &&
                     std::abs(last_ground_to_current_y) < c.ground_because_close_to_last_certain_ground_max_z_diff)
            {
                range_image_.ground_point_label[index] = GP_GROUND;
                range_image_.debug_ground_point_label[index] = YELLOW;
            }
        }

        // mark remaining points as obstacle
        if (range_image_.ground_point_label[index] != GP_GROUND)
        {
            range_image_.ground_point_label[index] = GP_OBSTACLE;
            range_image_.debug_ground_point_label[index] = RED;

            // go down in the rows and mark very close points also as obstacle
            int prev_row_index = row_index + 1;
            while (prev_row_index < range_image_.height)
            {
                uint64_t prev_index = range_image_.getIndex(col_idx, prev_row_index);

                // Convert previous point to 2D
                float prev_x_wrt_sensor = range_image_.x[prev_index] - sensor_position_point_.x;
                float prev_y_wrt_sensor = range_image_.y[prev_index] - sensor_position_point_.y;
                float prev_z_wrt_sensor = range_image_.z[prev_index] - sensor_position_point_.z;
                float xy_length_prev =
                    std::sqrt(prev_x_wrt_sensor * prev_x_wrt_sensor + prev_y_wrt_sensor * prev_y_wrt_sensor);

                float current_to_prev_x_diff = std::abs(xy_length_current - xy_length_prev);

                if (range_image_.debug_ground_point_label[prev_index] == YELLOW ||
                    (range_image_.ground_point_label[prev_index] == GP_GROUND &&
                     current_to_prev_x_diff < c.obstacle_because_next_certain_obstacle_max_dist_diff))
                {
                    if (range_image_.ground_point_label[prev_index] == GP_GROUND)
                    {
                        range_image_.ground_point_label[prev_index] = GP_OBSTACLE;
                        range_image_.debug_ground_point_label[prev_index] = DARKRED;
                    }
                    prev_row_index++;
                }
                else
                {
                    break;
                }
            }
        }

        // check whether we have ever seen an obstacle
        first_obstacle_detected |= range_image_.ground_point_label[index] == GP_OBSTACLE;

        // keep track of last (certain) ground point
        if (range_image_.debug_ground_point_label[index] == GREEN ||
            range_image_.debug_ground_point_label[index] == YELLOWGREEN)
        {
            // only use current point as the new last ground point when it was plausible. On wet streets there are often
            // false points below the ground surface because of reflections. Therefore, we do not want the slope to be
            // too much going down. Furthermore, in this case often there is a larger distance jump.
            if (slope_to_prev > c.last_ground_point_slope_higher_than &&
                std::abs(prev_to_current_x) < c.last_ground_point_distance_smaller_than && previous_label != YELLOW)
            {
                last_ground_x = current_x_wrt_sensor;
                last_ground_y = current_y_wrt_sensor;
                last_ground_z = current_z_wrt_sensor;
            }
        }

        // keep track of previous point
        previous_x = current_x_wrt_sensor;
        previous_y = current_y_wrt_sensor;
        previous_z = current_z_wrt_sensor;
        previous_label = range_image_.debug_ground_point_label[index];
    }

    // Process points for clustering - no changes needed for this part
    // Second pass to prepare for clustering
    for (int row_index = range_image_.height - 1; row_index >= 0; row_index--)
    {
        uint64_t index = range_image_.getIndex(col_idx, row_index);

        // prepare everything for next step in pipeline (point association)
        range_image_.is_ignored[index] = false;

        // ignore this point if it is NaN
        if (std::isnan(range_image_.distance[index]))
        {
            range_image_.is_ignored[index] = true;
            continue;
        }

        // only consider obstacle pixels
        if (range_image_.ground_point_label[index] != GP_OBSTACLE)
        {
            range_image_.is_ignored[index] = true;
            continue;
        }

        // ignore this pixel if it is too close
        if (range_image_.distance[index] < 1. * config_.clustering.max_distance)
        {
            range_image_.is_ignored[index] = true;
            continue;
        }

        // ignore this pixel if the distance in combination with elevation diff can't be below distance threshold
        // TODO: reenable this?
        // if (config_.clustering.ignore_pixels_with_too_big_elevation_angle_diff && row_index <
        // (range_image_.height - 1) &&
        //     std::atan2(config_.clustering.max_distance, pixel.distance) < laser_elevation_angles_[row_index])
        // {
        //     pixel.is_ignored = true;
        //     continue;
        // }

        // ignore pixels in a chessboard pattern
        if (config_.clustering.ignore_pixels_in_chessboard_pattern)
        {
            bool column_even = range_image_.monot_col_idx[index] % 2 == 0;
            bool row_even = row_index % 2 == 0;
            if ((column_even && !row_even) || (!column_even && row_even))
            {
                range_image_.is_ignored[index] = true;
                continue;
            }
        }
        else if (config_.clustering.ignore_pixels_in_every_second_row)
        {
            bool row_even = row_index % 2 == 0;
            if (row_even)
            {
                range_image_.is_ignored[index] = true;
                continue;
            }
        }
        else if (config_.clustering.ignore_pixels_in_every_second_column)
        {
            bool column_even = range_image_.monot_col_idx[index] % 2 == 0;
            if (column_even)
            {
                range_image_.is_ignored[index] = true;
                continue;
            }
        }
    }

    if (finished_column_callback_)
        finished_column_callback_(job.cur_monot_col_idx, job.cur_monot_col_idx, true);

    // lets enqueue the association job for this column to do it in a separate thread
    union_find_thread_pool_.enqueue({job.cur_monot_col_idx});
}

void ContinuousClustering::setTransformRobotFrameFromSensorFrame(const Eigen::Isometry3d& tf)
{
    if (!ego_robot_frame_from_sensor_frame_)
        ego_robot_frame_from_sensor_frame_ = std::make_unique<Eigen::Isometry3d>();
    *ego_robot_frame_from_sensor_frame_ = tf;
}

bool ContinuousClustering::hasTransformRobotFrameFromSensorFrame()
{
    return ego_robot_frame_from_sensor_frame_ != nullptr;
}

void ContinuousClustering::calculateFovBounds(int64_t& fov_start_monot_col_idx,
                                              int16_t& fov_start_row_idx,
                                              int16_t& fov_end_row_idx,
                                              uint16_t cur_row_idx,
                                              int64_t cur_monot_col_idx,
                                              float half_angular_fov)
{
    // left bound
    int required_steps_back = static_cast<int>(std::ceil(half_angular_fov / azimuth_width_per_column_));
    fov_start_monot_col_idx = std::max(ring_buf_start_monot_col_idx_, cur_monot_col_idx - required_steps_back);

    // optain elevation angle of current pixel
    double cur_elevation_angle = laser_elevation_angles_[cur_row_idx];

    // upper bound
    uint16_t row_idx = cur_row_idx;
    while (row_idx > 0 && laser_elevation_angles_[row_idx - 1] < cur_elevation_angle + half_angular_fov)
        row_idx--;
    fov_start_row_idx = row_idx;

    // lower bound
    row_idx = cur_row_idx;
    while (row_idx < range_image_.height - 1 &&
           laser_elevation_angles_[row_idx + 1] > cur_elevation_angle - half_angular_fov)
        row_idx++;
    fov_end_row_idx = row_idx;
}

bool ContinuousClustering::findEdgesInFieldOfView(uint64_t pixel_idx,
                                                  int64_t monot_col_idx,
                                                  uint16_t row_idx,
                                                  float half_angular_fov)
{
    // calculate FoV bounds
    int64_t fov_start_monot_col_idx;
    int16_t fov_start_row_idx;
    int16_t fov_end_row_idx;
    calculateFovBounds(
        fov_start_monot_col_idx, fov_start_row_idx, fov_end_row_idx, row_idx, monot_col_idx, half_angular_fov);

    // Vector to store indices of pixels that are below the distance threshold
    std::vector<uint64_t> potential_neighbors;

    // Phase 1: Calculate distances and find potential neighbors (can be vectorized by compiler)
    for (int64_t other_monot_col_idx = monot_col_idx; other_monot_col_idx >= fov_start_monot_col_idx;
         other_monot_col_idx--)
    {
        for (uint16_t other_row_idx = fov_start_row_idx; other_row_idx <= fov_end_row_idx; other_row_idx++)
        {
            if (other_monot_col_idx == monot_col_idx && other_row_idx >= row_idx)
                continue;

            // get other pixel
            uint16_t other_col_idx = range_image_.fromMonotColIdx(other_monot_col_idx);
            uint64_t pixel_other_idx = range_image_.getIndex(other_col_idx, other_row_idx);
            
            // count number of visited pixels for analyzing
            // range_image_.number_of_visited_neighbors[pixel_other_idx] += 1;
            
            // if other pixel is not ignored and is below the clustering threshold -> save for association
            if (!range_image_.is_ignored[pixel_other_idx] &&
                range_image_.lengthSquared(pixel_idx, pixel_other_idx) < max_distance_squared_)
            {
                potential_neighbors.push_back(pixel_other_idx);
            }
        }
    }
    
    // Phase 2: Perform union operations on potential neighbors
    bool at_least_one_edge = false;
    for (uint64_t pixel_other_idx : potential_neighbors)
    {
        at_least_one_edge |= union_set(pixel_idx, pixel_other_idx);
    }
    
    return at_least_one_edge;
}

void ContinuousClustering::performUnionFindForColumn(UnionFindJob&& job)
{
    // clear all columns that are not needed anymore
    int64_t prev_ring_buf_start_monot_col_idx = ring_buf_start_monot_col_idx_;
    ring_buf_start_monot_col_idx_ = min_unfinished_monot_col_idx_;
    range_image_.clearColumns(prev_ring_buf_start_monot_col_idx, ring_buf_start_monot_col_idx_ - 1);

    // get actual column index of current column
    uint16_t col_idx = range_image_.fromMonotColIdx(job.cur_monot_col_idx);

    for (int row_index = 0; row_index < range_image_.height; row_index++)
    {
        // Get index for current pixel in the range image
        uint64_t pixel_idx = range_image_.getIndex(col_idx, row_index);

        // check whether pixel should be ignored
        if (range_image_.is_ignored[pixel_idx])
            continue;

        // calculate minimum required angle diff to consider at which no further pixel can be linked to this pixel
        float half_angular_fov = std::asin(config_.clustering.max_distance / range_image_.distance[pixel_idx]);

        // initialize a new cluster containing only this pixel (initialize for union find)
        make_set(pixel_idx, half_angular_fov);

        // traverse field of view
        bool neighbor_found = findEdgesInFieldOfView(pixel_idx, job.cur_monot_col_idx, row_index, half_angular_fov);
        if (!neighbor_found)
        {
            range_image_.is_potential_cluster_root[pixel_idx] = true;
            potential_cluster_root_idxs_.push_back(pixel_idx);
        }
        else
        {
            range_image_.is_potential_cluster_root[pixel_idx] = false;
        }
    }

    // it has to run in the same thread as both access/modify list of potential cluster roots!
    identifyFinishedClusters(job.cur_monot_col_idx);
}

void ContinuousClustering::identifyFinishedClusters(int64_t cur_monot_col_idx)
{
    // keep track of the minimum column index of all unfinished clusters
    int64_t minimum_required_monot_col_idx = std::numeric_limits<int64_t>::max();

    // split into “finished" and "unfinished" clusters
    std::vector<uint64_t> finished_cluster_root_idxs;
    std::vector<uint64_t> unfinished_cluster_root_idxs;

    // calculate monotonic azimuth angle of current column
    double monot_azimuth_angle_of_col = cur_monot_col_idx * static_cast<double>(azimuth_width_per_column_);

    // iterate over potential cluster roots
    for (uint64_t root_pixel_idx : potential_cluster_root_idxs_)
    {
        // discard cluster roots eliminated during union operation
        if (!range_image_.is_potential_cluster_root[root_pixel_idx])
            continue;

        // check whether no more points can be added to this cluster
        if (monot_azimuth_angle_of_col > range_image_.finished_at_monot_azimuth_angle[root_pixel_idx])
        {
            finished_cluster_root_idxs.push_back(root_pixel_idx);
        }
        else
        {
            unfinished_cluster_root_idxs.push_back(root_pixel_idx);
            minimum_required_monot_col_idx =
                std::min(minimum_required_monot_col_idx, range_image_.clust_start_monot_col_idx[root_pixel_idx]);
        }
    }

    // replace the old list of potential cluster roots
    potential_cluster_root_idxs_ = unfinished_cluster_root_idxs;

    // if no unfinished clusters, set start index one after current column
    if (minimum_required_monot_col_idx == std::numeric_limits<int64_t>::max())
        minimum_required_monot_col_idx = cur_monot_col_idx + 1;

    PointCollectionJob next_job;
    next_job.cur_monot_col_idx = cur_monot_col_idx;
    next_job.min_required_monot_col_idx = minimum_required_monot_col_idx;
    next_job.cluster_root_idxs = std::move(finished_cluster_root_idxs);
    point_collection_thread_pool_.enqueue(std::move(next_job));
}

void ContinuousClustering::collectPointsForCusterAndPublish(PointCollectionJob&& job)
{
    // keep track of minimum stamp for this message
    uint64_t min_stamp_for_this_msg = std::numeric_limits<uint64_t>::max();

    // create buffer
    std::vector<uint64_t> pixels_idxs_of_cluster;

    for (uint64_t cluster_root_idx : job.cluster_root_idxs)
    {
        // create cluster id
        uint64_t cluster_id =
            range_image_.monot_col_idx[cluster_root_idx] * range_image_.height + range_image_.row_idx[cluster_root_idx];

        // collect minimum and maximum stamp for this cluster
        uint64_t min_stamp_for_this_cluster = std::numeric_limits<uint64_t>::max();
        uint64_t max_stamp_for_this_cluster = 0;

        // collect all of its child pixels
        // extension for print after union find
        uint64_t pixel_idx = cluster_root_idx;
        while (true)
        {
            if (range_image_.stamp_ns[pixel_idx] < min_stamp_for_this_cluster)
                min_stamp_for_this_cluster = range_image_.stamp_ns[pixel_idx];
            if (range_image_.stamp_ns[pixel_idx] > max_stamp_for_this_cluster)
                max_stamp_for_this_cluster = range_image_.stamp_ns[pixel_idx];
            range_image_.id[pixel_idx] = cluster_id;
            pixels_idxs_of_cluster.push_back(pixel_idx);

            pixel_idx = range_image_.next_idx[pixel_idx];
            if (pixel_idx == cluster_root_idx)
                break;
        }

        // keep track of minimum stamp for this message
        if (min_stamp_for_this_cluster < min_stamp_for_this_msg)
            min_stamp_for_this_msg = min_stamp_for_this_cluster;

        // publish pixels (TODO: make threshold configurable)
        if (finished_cluster_callback_)
        {
            uint64_t stamp_cluster =
                config_.clustering.use_last_point_for_cluster_stamp ?
                    max_stamp_for_this_cluster :
                    min_stamp_for_this_cluster + (max_stamp_for_this_cluster - min_stamp_for_this_cluster) / 2;
            finished_cluster_callback_(pixels_idxs_of_cluster, stamp_cluster);
        }
    }

    if (finished_column_callback_)
        finished_column_callback_(min_unfinished_monot_col_idx_, job.min_required_monot_col_idx - 1, false);
    min_unfinished_monot_col_idx_ = job.min_required_monot_col_idx;

    // the cols are not cleared here but in the edge generation/association step
}

void ContinuousClustering::recordJobQueueWorkload(uint64_t num_jobs_sensor_input)
{
    if (stop_statistics_)
        return;
    num_pending_jobs_.push_back(num_jobs_sensor_input);
    num_pending_jobs_.push_back(range_image_thread_pool_.getNumberOfUnprocessedJobs());
    num_pending_jobs_.push_back(ground_segmentation_thread_pool_.getNumberOfUnprocessedJobs());
    num_pending_jobs_.push_back(union_find_thread_pool_.getNumberOfUnprocessedJobs());
    num_pending_jobs_.push_back(point_collection_thread_pool_.getNumberOfUnprocessedJobs());
    // Keep the list at a reasonable size by removing elements from the front when it gets too large
    while (num_pending_jobs_.size() > 100000 * 5)
        num_pending_jobs_.pop_front();
}

void ContinuousClustering::make_set(uint64_t pixel_idx, float half_angular_fov)
{
    // Regular union find algorithm
    range_image_.parent_idx[pixel_idx] = pixel_idx;
    range_image_.rank[pixel_idx] = 0;

    // Extension for collecting pixels after union find
    range_image_.next_idx[pixel_idx] = pixel_idx;

    // Extension for finished cluster extraction
    range_image_.finished_at_monot_azimuth_angle[pixel_idx] =
        range_image_.monot_azimuth_angle[pixel_idx] + half_angular_fov;
    range_image_.is_potential_cluster_root[pixel_idx] = false;

    // Infinite cluster detection (e.g. in a closed room or tunnel)
    range_image_.clust_start_monot_col_idx[pixel_idx] = range_image_.monot_col_idx[pixel_idx];
    range_image_.clust_end_monot_col_idx[pixel_idx] = range_image_.monot_col_idx[pixel_idx];
}

uint64_t ContinuousClustering::find_set(uint64_t pixel_idx)
{
    // Regular union find algorithm with path compression
    // Find root index of current tree
    uint64_t root_idx = pixel_idx;
    while (range_image_.parent_idx[root_idx] != root_idx)
        root_idx = range_image_.parent_idx[root_idx];

    // Path compression: iterate from leaf to root and re-attach all indices directly to root
    uint64_t current_idx = pixel_idx;
    while (range_image_.parent_idx[current_idx] != root_idx)
    {
        uint64_t next_idx = range_image_.parent_idx[current_idx];
        range_image_.parent_idx[current_idx] = root_idx;
        current_idx = next_idx;
    }

    return root_idx;
}

bool ContinuousClustering::union_set(uint64_t pixel_a_idx, uint64_t pixel_b_idx)
{
    // Regular union find algorithm
    uint64_t root_a_idx = find_set(pixel_a_idx);
    uint64_t root_b_idx = find_set(pixel_b_idx);

    if (root_a_idx == root_b_idx)
        return true; // Already same cluster -> nothing to do

    // Extension for infinite cluster detection
    int64_t new_start_col_idx = std::min(range_image_.clust_start_monot_col_idx[root_a_idx],
                                         range_image_.clust_start_monot_col_idx[root_b_idx]);
    int64_t new_end_col_idx =
        std::max(range_image_.clust_end_monot_col_idx[root_a_idx], range_image_.clust_end_monot_col_idx[root_b_idx]);
    int new_width = new_end_col_idx - new_start_col_idx + 1;
    if (new_width > num_columns_rot_)
        return false; // Clusters not merged (broader than full rotation)

    // Regular union find algorithm with union by rank
    uint64_t root_after_union_idx;
    uint64_t child_after_union_idx;
    if (range_image_.rank[root_a_idx] > range_image_.rank[root_b_idx])
    {
        range_image_.parent_idx[root_b_idx] = root_a_idx;
        root_after_union_idx = root_a_idx;
        child_after_union_idx = root_b_idx;
    }
    else
    {
        range_image_.parent_idx[root_a_idx] = root_b_idx;
        if (range_image_.rank[root_a_idx] == range_image_.rank[root_b_idx])
            range_image_.rank[root_b_idx]++;
        root_after_union_idx = root_b_idx;
        child_after_union_idx = root_a_idx;
    }

    // Extension for infinite cluster detection
    range_image_.clust_start_monot_col_idx[root_after_union_idx] = new_start_col_idx;
    range_image_.clust_end_monot_col_idx[root_after_union_idx] = new_end_col_idx;

    // Extension for cluster extraction
    range_image_.finished_at_monot_azimuth_angle[root_after_union_idx] =
        std::max(range_image_.finished_at_monot_azimuth_angle[root_a_idx],
                 range_image_.finished_at_monot_azimuth_angle[root_b_idx]);
    range_image_.is_potential_cluster_root[child_after_union_idx] = false;

    // Extension for collecting pixels (swap next indices)
    uint64_t tmp = range_image_.next_idx[root_b_idx];
    range_image_.next_idx[root_b_idx] = range_image_.next_idx[root_a_idx];
    range_image_.next_idx[root_a_idx] = tmp;

    return true;
}

} // namespace continuous_clustering
