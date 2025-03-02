#include <continuous_clustering/clustering/continuous_clustering.hpp>

#include <iostream>
#include <utility>

namespace continuous_clustering
{

ContinuousClustering::ContinuousClustering() = default;

void ContinuousClustering::reset(int num_rows)
{
    // recalculate some intermediate values in case the parameters have changed
    num_rows_ = num_rows;
    num_columns_rot_ = config_.range_image.num_columns_rot;
    num_columns_ = num_columns_rot_ * 10;
    azimuth_width_per_column_ = static_cast<float>((2 * M_PI)) / static_cast<float>(num_columns_rot_);

    // shutdown workers
    range_image_thread_pool_.shutdown();
    ground_segmentation_thread_pool_.shutdown();
    union_find_thread_pool_.shutdown();
    point_collection_thread_pool_.shutdown();

    // init/reset range image (implemented as ring buffer)
    range_image_.resize(num_columns_ * num_rows);
    clearColumns(0, num_columns_ - 1);
    ring_buf_start_monot_col_idx_ = -1; // does not start at zero but at the minimum laser of first firing
    ring_buf_end_monot_col_idx_ = -1;

    // reset members for continuous range image generation
    min_incomlete_monot_col_idx_ = 0;
    reset_required_ = false;

    // reset members for continuous ground point segmentation (sgps)
    ego_robot_frame_from_sensor_frame_.reset();

    // reset members for continuous clustering
    min_unfinished_monot_col_idx_ = -1;
    elevation_angles_between_lasers_.resize(num_rows, std::nanf(""));
    potential_cluster_roots_.clear();

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
    if (num_rows_ != firing->points.size())
        throw std::runtime_error("The number of points in a firing has changed. This is probably a bug!");
    range_image_thread_pool_.enqueue({firing, odom_from_sensor});
}

void ContinuousClustering::setFinishedColumnCallback(std::function<void(int64_t, int64_t, bool)> cb)
{
    finished_column_callback_ = std::move(cb);
}

void ContinuousClustering::setFinishedClusterCallback(std::function<void(const std::vector<Pixel>&, uint64_t)> cb)
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
        int col_idx = static_cast<int>(monot_col_idx % num_columns_);

        // get correct pixel
        Pixel* pixel = &range_image_[col_idx * num_rows_ + row_idx]; // column major order

        // calculate continuous azimuth angle (even if we move it to the next cell, this value remains the same)
        double monot_azimuth_angle = (2 * M_PI) * static_cast<double>(rotation_index) + increasing_azimuth_angle;

        // in case this cell is already occupied, try next column
        auto distance = static_cast<float>(p.norm());
        if (!std::isnan(pixel->distance) && !std::isnan(distance))
        {
            int next_col_idx = col_idx + 1;
            if (next_col_idx >= num_columns_)
                next_col_idx -= num_columns_;
            Pixel* next_pixel = &range_image_[next_col_idx * num_rows_ + row_idx];
            if (std::isnan(next_pixel->distance))
            {
                pixel = next_pixel;
                col_idx = next_col_idx;
                monot_col_idx++;
            }
        }

        // avoid that a valid cell (non-nan) is overwritten by a nan or more distant value
        if (!std::isnan(pixel->distance) && (std::isnan(distance) || distance >= pixel->distance))
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

            // fill pixel data
            pixel->xyz.x = static_cast<float>(p_odom.x()); // TODO: make transform at a later point in time?
            pixel->xyz.y = static_cast<float>(p_odom.y());
            pixel->xyz.z = static_cast<float>(p_odom.z());
            pixel->firing_idx = raw_point.firing_index;
            pixel->intensity = raw_point.intensity;
            pixel->stamp_ns = raw_point.stamp;
            pixel->distance = distance;
            pixel->azimuth_angle = azimuth_angle;
            pixel->elevation_angle = std::asin(static_cast<float>(p.z()) / pixel->distance);
            pixel->monot_azimuth_angle = monot_azimuth_angle; // omitted cells will be filled again later
            pixel->col_idx = col_idx;
            pixel->row_idx = row_idx;
            pixel->monot_col_idx = monot_col_idx; // omitted cells will be filled again later
            pixel->globally_unique_point_index = raw_point.globally_unique_point_index;
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
        ground_segmentation_thread_pool_.enqueue({min_incomlete_monot_col_idx_++, job.odom_frame_from_sensor_frame});
}

void ContinuousClustering::performGroundPointSegmentationForColumn(SegmentationJob&& job)
{
    int col_idx = static_cast<int>(job.current_monot_col_idx % num_columns_);

    if (!ego_robot_frame_from_sensor_frame_)
        throw std::runtime_error("Transform robot frame from sensor frame was not set yet!");
    Eigen::Isometry3d ego_robot_frame_from_odom_frame =
        *ego_robot_frame_from_sensor_frame_ * job.odom_frame_from_sensor_frame.inverse();
    float height_sensor_to_ground = -static_cast<float>(ego_robot_frame_from_sensor_frame_->translation().z()) +
                                    config_.ground_segmentation.height_ref_to_ground_;

    // iterate rows from bottom to top and find ground points
    bool first_obstacle_detected = false;
    bool first_point_found = false;
    Point3D last_ground_position_wrt_sensor{0, 0, height_sensor_to_ground};
    Point3D ground_x_meter_behind_position_wrt_sensor = {0, 0, 0};
    Point3D previous_position_wrt_sensor;
    uint8_t previous_label;
    float elevation_previous_laser = 0; // calculate difference between elevation angles for subsequent steps

    for (int row_index = num_rows_ - 1; row_index >= 0; row_index--)
    {
        // obtain pixel
        Pixel& pixel = range_image_[col_idx * num_rows_ + row_index];

        // check if there is a problem with the ring buffer
        int64_t pixel_monot_col_idx_copy = pixel.monot_col_idx;
        if (pixel_monot_col_idx_copy != job.current_monot_col_idx && pixel_monot_col_idx_copy != -1)
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
                std::to_string(pixel_monot_col_idx_copy) + ", " + std::to_string(job.current_monot_col_idx) + ", " +
                std::to_string(num_columns_) +
                "; This typically happens when the clustering is not fast enough to handle all the firings. Consider "
                "to play the sensor data more slowly or to adjust the parameters to make the clustering faster.");
        }

        // refill local/global column index because it was not filled for omitted cells
        pixel.monot_col_idx = job.current_monot_col_idx;
        pixel.col_idx = static_cast<int>(job.current_monot_col_idx % num_columns_);

        // keep track of (differences between) the elevation angles of the lasers (for later processing steps)
        float elevation_current_laser = range_image_[col_idx * num_rows_ + row_index].elevation_angle;
        float diff = elevation_current_laser - elevation_previous_laser;
        if (!std::isnan(diff))
            elevation_angles_between_lasers_[row_index] = diff; // last value is useless but we do not use it
        elevation_previous_laser = elevation_current_laser;

        // skip NaN's
        if (std::isnan(pixel.distance))
        {
            // use elevation angle from previous column (it is useful to supplement nan cells with an elevation
            // angle in order to be able to break the while loop earlier during association
            if (config_.range_image.supplement_elevation_angle_for_nan_cells && row_index < num_rows_ - 1)
            {
                Pixel& pixel_below = range_image_[col_idx * num_rows_ + (row_index + 1)];
                pixel.elevation_angle = pixel_below.elevation_angle + elevation_angles_between_lasers_[row_index];
            }
            // recalculate continuous azimuth for omitted/NaN cells (for later processing steps)
            pixel.monot_azimuth_angle =
                (static_cast<double>(job.current_monot_col_idx) + 0.5) * azimuth_width_per_column_;
            continue;
        }

        // skip pixels which seem to be fog
        if (config_.ground_segmentation.fog_filtering_enabled &&
            pixel.intensity < config_.ground_segmentation.fog_filtering_intensity_below &&
            pixel.distance < config_.ground_segmentation.fog_filtering_distance_below &&
            pixel.elevation_angle > config_.ground_segmentation.fog_filtering_elevation_above)
        {
            pixel.ground_point_label = GP_FOG;
            pixel.debug_ground_point_label = LIGHTGRAY;
            continue;
        }

        const Point3D& current_position = pixel.xyz;

        // special handling for points on ego vehicle surface
        Eigen::Vector3d current_position_in_ego_robot_frame =
            ego_robot_frame_from_odom_frame *
            Eigen::Vector3d(current_position.x, current_position.y, current_position.z);
        const auto& c = config_.ground_segmentation;
        if (current_position_in_ego_robot_frame.x() < c.length_ref_to_front_end_ &&
            current_position_in_ego_robot_frame.x() > c.length_ref_to_rear_end_ &&
            current_position_in_ego_robot_frame.y() < c.width_ref_to_left_mirror_ &&
            current_position_in_ego_robot_frame.y() > c.width_ref_to_right_mirror_ &&
            current_position_in_ego_robot_frame.z() < c.height_ref_to_maximum_ &&
            current_position_in_ego_robot_frame.z() > c.height_ref_to_ground_)
        {
            pixel.ground_point_label = GP_EGO_VEHICLE;
            pixel.debug_ground_point_label = VIOLET;
            continue;
        }

        Point3D current_position_wrt_sensor = current_position - sensor_position_point_;

        // special handling first point outside the ego bounding box
        if (!first_point_found)
        {
            // now we found the first point outside the ego vehicle box
            first_point_found = true;
            float height_over_predicted_ground = current_position_wrt_sensor.z - height_sensor_to_ground;
            if (height_over_predicted_ground > c.first_ring_as_ground_min_allowed_z_diff &&
                height_over_predicted_ground < c.first_ring_as_ground_max_allowed_z_diff)
            {
                pixel.ground_point_label = GP_GROUND;
                pixel.debug_ground_point_label = GRAY;
                last_ground_position_wrt_sensor = current_position_wrt_sensor;
                first_obstacle_detected = false;
            }
            else
            {
                pixel.ground_point_label = GP_OBSTACLE;
                pixel.debug_ground_point_label = ORANGE;
                first_obstacle_detected = true;
            }
            previous_position_wrt_sensor = current_position_wrt_sensor;
            previous_label = pixel.debug_ground_point_label;
            continue;
        }

        // calculate the slope w.r.t previous point
        Point2D current_position_wrt_sensor_2d = to2dInAzimuthPlane(current_position_wrt_sensor);
        Point2D previous_position_wrt_sensor_2d = to2dInAzimuthPlane(previous_position_wrt_sensor);
        Point2D previous_to_current = current_position_wrt_sensor_2d - previous_position_wrt_sensor_2d;
        float slope_to_prev = previous_to_current.y / previous_to_current.x;
        bool is_flat_wrt_prev = std::abs(slope_to_prev) < c.max_slope && previous_to_current.x > 0;

        // calculate slope w.r.t. last seen (quite certain) ground point
        Point2D last_ground_position_wrt_sensor_2d = to2dInAzimuthPlane(last_ground_position_wrt_sensor);
        Point2D last_ground_to_current = current_position_wrt_sensor_2d - last_ground_position_wrt_sensor_2d;
        float slope_to_last_ground = last_ground_to_current.y / last_ground_to_current.x;
        bool is_flat_wrt_last_ground = std::abs(slope_to_last_ground) < c.max_slope && last_ground_to_current.x > 0;

        // quite certain ground points
        if (!first_obstacle_detected && is_flat_wrt_prev)
        {
            pixel.ground_point_label = GP_GROUND;
            pixel.debug_ground_point_label = GREEN;
        }
        else // try to find remaining ground points
        {
            if (first_obstacle_detected && is_flat_wrt_prev && is_flat_wrt_last_ground)
            {
                pixel.ground_point_label = GP_GROUND;
                pixel.debug_ground_point_label = YELLOWGREEN;
            }
            else if (std::abs(last_ground_to_current.x) < c.ground_because_close_to_last_certain_ground_max_dist_diff &&
                     std::abs(last_ground_to_current.y) < c.ground_because_close_to_last_certain_ground_max_z_diff)
            {
                pixel.ground_point_label = GP_GROUND;
                pixel.debug_ground_point_label = YELLOW;
            }
        }

        // mark remaining points as obstacle
        if (pixel.ground_point_label != GP_GROUND)
        {
            pixel.ground_point_label = GP_OBSTACLE;
            pixel.debug_ground_point_label = RED;

            // go down in the rows and mark very close points also as obstacle
            int prev_row_index = row_index + 1;
            while (prev_row_index < num_rows_)
            {
                Pixel& cur_pixel = range_image_[col_idx * num_rows_ + prev_row_index];
                Point2D prev_position_wrt_sensor_2d = to2dInAzimuthPlane(cur_pixel.xyz - sensor_position_point_);
                if (cur_pixel.debug_ground_point_label == YELLOW ||
                    (cur_pixel.ground_point_label == GP_GROUND &&
                     std::abs((current_position_wrt_sensor_2d - prev_position_wrt_sensor_2d).x) <
                         c.obstacle_because_next_certain_obstacle_max_dist_diff))
                {
                    if (cur_pixel.ground_point_label == GP_GROUND)
                    {
                        cur_pixel.ground_point_label = GP_OBSTACLE;
                        cur_pixel.debug_ground_point_label = DARKRED;
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
        first_obstacle_detected |= pixel.ground_point_label == GP_OBSTACLE;

        // keep track of last (certain) ground point
        if (pixel.debug_ground_point_label == GREEN || pixel.debug_ground_point_label == YELLOWGREEN)
        {
            // only use current point as the new last ground point when it was plausible. On wet streets there are often
            // false points below the ground surface because of reflections. Therefore, we do not want the slope to be
            // too much going down. Furthermore, in this case often there is a larger distance jump.
            if (slope_to_prev > c.last_ground_point_slope_higher_than &&
                std::abs(previous_to_current.x) < c.last_ground_point_distance_smaller_than && previous_label != YELLOW)
            {
                last_ground_position_wrt_sensor = current_position_wrt_sensor;
            }
        }

        // keep track of previous point
        previous_position_wrt_sensor = current_position_wrt_sensor;
        previous_label = pixel.debug_ground_point_label;
    }

    for (int row_index = num_rows_ - 1; row_index >= 0; row_index--)
    {
        int current_data_index_ri = col_idx * num_rows_ + row_index; // column major
        Pixel& pixel = range_image_[current_data_index_ri];

        // prepare everything for next step in pipeline (point association)
        pixel.is_ignored = false;

        // ignore this point if it is NaN
        if (std::isnan(pixel.distance))
        {
            pixel.is_ignored = true;
            continue;
        }

        // only consider obstacle pixels
        if (pixel.ground_point_label != GP_OBSTACLE)
        {
            pixel.is_ignored = true;
            continue;
        }

        // ignore this pixel if it is too close
        if (pixel.distance < 1. * config_.clustering.max_distance)
        {
            pixel.is_ignored = true;
            continue;
        }

        // ignore this pixel if the distance in combination with elevation diff can't be below distance threshold
        if (config_.clustering.ignore_pixels_with_too_big_elevation_angle_diff && row_index < (num_rows_ - 1) &&
            std::atan2(config_.clustering.max_distance, pixel.distance) < elevation_angles_between_lasers_[row_index])
        {
            pixel.is_ignored = true;
            continue;
        }

        // ignore pixels in a chessboard pattern
        if (config_.clustering.ignore_pixels_in_chessboard_pattern)
        {
            bool column_even = pixel.monot_col_idx % 2 == 0;
            bool row_even = row_index % 2 == 0;
            if ((column_even && !row_even) || (!column_even && row_even))
            {
                pixel.is_ignored = true;
                continue;
            }
        }
        else if (config_.clustering.ignore_pixels_in_every_second_row)
        {
            bool row_even = row_index % 2 == 0;
            if (row_even)
            {
                pixel.is_ignored = true;
                continue;
            }
        }
        else if (config_.clustering.ignore_pixels_in_every_second_column)
        {
            bool column_even = pixel.monot_col_idx % 2 == 0;
            if (column_even)
            {
                pixel.is_ignored = true;
                continue;
            }
        }
    }

    if (finished_column_callback_)
        finished_column_callback_(job.current_monot_col_idx, job.current_monot_col_idx, true);

    // lets enqueue the association job for this column to do it in a separate thread
    union_find_thread_pool_.enqueue({job.current_monot_col_idx});
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

bool ContinuousClustering::checkClusteringCondition(const Pixel& pixel_a, const Pixel& pixel_b) const
{
    return (pixel_a.xyz - pixel_b.xyz).lengthSquared() < max_distance_squared_;
}

void ContinuousClustering::make_set(Pixel* pixel, float half_angular_fov)
{
    // regular union find algorithm
    pixel->parent = pixel;
    pixel->rank = 0;

    // extension for collect after union find
    pixel->next = pixel;

    // extension for finished cluster extraction
    pixel->finished_at_monot_azimuth_angle = pixel->monot_azimuth_angle + half_angular_fov;
    pixel->is_potential_cluster_root = false;

    // infinite cluster detection (e.g. in a closed room or tunnel)
    pixel->clust_start_monot_col_idx = pixel->monot_col_idx;
    pixel->clust_end_monot_col_idx = pixel->monot_col_idx;
}

Pixel* ContinuousClustering::find_set(Pixel* pixel)
{
    // regular union find algorithm with path compression

    // find root vertex of current tree
    Pixel* root = pixel;
    while (root->parent != root)
        root = root->parent;

    // path compression: again, iterate from leaf to root and
    // re-attach all vertices to the (now known) root
    while (pixel->parent != root)
    {
        Pixel* tmp = pixel->parent;
        pixel->parent = root;
        pixel = tmp;
    }

    return root;
}

bool ContinuousClustering::union_set(Pixel* pixel_a, Pixel* pixel_b)
{
    // regular union find algorithm
    Pixel* root_a = find_set(pixel_a);
    Pixel* root_b = find_set(pixel_b);
    if (root_a == root_b)
        return true; // already same cluster -> nothing to do

    // extension for infinite cluster detection
    int64_t new_start_col_idx = std::min(root_a->clust_start_monot_col_idx, root_b->clust_start_monot_col_idx);
    int64_t new_end_col_idx = std::max(root_a->clust_end_monot_col_idx, root_b->clust_end_monot_col_idx);
    int new_width = new_end_col_idx - new_start_col_idx + 1;
    if (new_width > num_columns_rot_)
        return false; // clusters not merged (broader than full rotation)

    // regular union find algorithm (with union by rank)
    Pixel* root_after_union;
    Pixel* child_after_union;
    if (root_a->rank > root_b->rank)
    {
        root_b->parent = root_a;
        root_after_union = root_a;
        child_after_union = root_b;
    }
    else
    {
        root_a->parent = root_b;
        if (root_a->rank == root_b->rank)
            root_b->rank += 1;
        root_after_union = root_b;
        child_after_union = root_a;
    }

    // extension for infinite cluster detection
    root_after_union->clust_start_monot_col_idx = new_start_col_idx;
    root_after_union->clust_end_monot_col_idx = new_end_col_idx;

    // extension for cluster extraction
    root_after_union->finished_at_monot_azimuth_angle =
        std::max(root_a->finished_at_monot_azimuth_angle, root_b->finished_at_monot_azimuth_angle);
    child_after_union->is_potential_cluster_root = false;

    // extension for collecting pixels (swap next pointers)
    Pixel* tmp = root_b->next;
    root_b->next = root_a->next;
    root_a->next = tmp;

    return true;
}

void ContinuousClustering::print_set(Pixel* pixel, std::vector<Pixel>& v)
{
    // extension for print after union find
    v.clear();
    Pixel* start_pixel = pixel;
    v.push_back(*pixel);
    while (pixel->next != start_pixel)
    {
        pixel = pixel->next;
        v.push_back(*pixel);
    }
}

bool ContinuousClustering::findEdgesInFieldOfView(Pixel& pixel, float half_angular_fov, int ring_buf_first_col_idx)
{
    // go left each column until azimuth angle difference gets too large
    bool at_least_one_edge = false;
    int required_steps_back = static_cast<int>(std::ceil(half_angular_fov / azimuth_width_per_column_));
    required_steps_back = std::min(required_steps_back, config_.clustering.max_steps_in_row);
    int64_t other_col_idx = pixel.col_idx;
    for (int num_steps_back = 0; num_steps_back <= required_steps_back; num_steps_back++)
    {
        for (int direction = -1; direction <= 1; direction += 2)
        {
            // do not go down in first column (these pixels are not associated to tree yet!)
            if (direction == 1 && num_steps_back == 0)
                continue;

            // go up/down each row until the elevation angle difference gets too large
            int num_steps_vertical = direction == 1 || num_steps_back == 0 ? 1 : 0;
            int other_row_index = direction == 1 || num_steps_back == 0 ? pixel.row_idx + direction : pixel.row_idx;
            while (other_row_index >= 0 && other_row_index < num_rows_ &&
                   num_steps_vertical <= config_.clustering.max_steps_in_column)
            {
                // get other pixel
                Pixel& pixel_other = range_image_[other_col_idx * num_rows_ + other_row_index];

                // count number of visited pixels for analyzing
                pixel.number_of_visited_neighbors += 1;

                // no cluster can be associated because the elevation angle diff gets too large
                if (std::abs(pixel_other.elevation_angle - pixel.elevation_angle) > half_angular_fov)
                    break;

                // if other pixel is ignored or has already the same tree root then do nothing (*1)
                if (!pixel_other.is_ignored &&
                    checkClusteringCondition(pixel, pixel_other)) // TODO: UF make find before clustering condition?
                {
                    at_least_one_edge = union_set(&pixel, &pixel_other);
                }

                // stop searching if pixel was already associated and minimum number of cols were processed
                if (pixel.parent != &pixel && config_.clustering.stop_after_first_edge_enabled &&
                    num_steps_vertical >= config_.clustering.stop_after_first_edge_min_steps)
                    break;

                other_row_index += direction;
                num_steps_vertical++;
            }
        }

        // stop searching if pixel was already associated and minimum number of pixels were processed
        if (pixel.parent != &pixel && config_.clustering.stop_after_first_edge_enabled &&
            num_steps_back >= config_.clustering.stop_after_first_edge_min_steps)
            break;

        // stop searching if we are at the beginning of the ring buffer
        if (other_col_idx == ring_buf_first_col_idx)
            break;

        other_col_idx--;

        // jump to the end of the ring buffer
        if (other_col_idx < 0)
            other_col_idx += num_columns_;
    }

    return at_least_one_edge;
}

void ContinuousClustering::performUnionFindForColumn(UnionFindJob&& job)
{
    // clear all columns that are not needed anymore
    int64_t prev_ring_buf_start_monot_col_idx = ring_buf_start_monot_col_idx_;
    ring_buf_start_monot_col_idx_ = min_unfinished_monot_col_idx_;
    clearColumns(prev_ring_buf_start_monot_col_idx, ring_buf_start_monot_col_idx_ - 1);

    // keep track of the minimum azimuth angle of all points in the current column
    double min_monot_azimuth_angle_in_col = std::numeric_limits<double>::max();

    // get actual start index of ring buffer start (used lower bound to calculate left edge of FoV window)
    int ring_buf_first_col_idx = static_cast<int>(min_unfinished_monot_col_idx_ % num_columns_);

    // get actual column index of current column
    int current_col_idx = static_cast<int>(job.current_monot_col_idx % num_columns_);

    for (int row_index = 0; row_index < num_rows_; row_index++)
    {
        // get current pixel (range image is stored in column-major order)
        Pixel& pixel = range_image_[current_col_idx * num_rows_ + row_index];

        // keep track of the current minimum continuous azimuth angle
        min_monot_azimuth_angle_in_col = std::min(min_monot_azimuth_angle_in_col, pixel.monot_azimuth_angle);

        // check whether pixel should be ignored
        if (pixel.is_ignored)
            continue;

        // calculate minimum required angle diff to consider at which no further pixel can be linked to this pixel
        float half_angular_fov = std::asin(config_.clustering.max_distance / pixel.distance);

        // initialize a new cluster containing only this pixel (initialize for union find)
        make_set(&pixel, half_angular_fov);

        // traverse field of view
        bool neighbor_found = findEdgesInFieldOfView(pixel, half_angular_fov, ring_buf_first_col_idx);
        if (!neighbor_found)
        {
            pixel.is_potential_cluster_root = true;
            potential_cluster_roots_.push_back(&pixel);
        }
        else
        {
            pixel.is_potential_cluster_root = false;
        }
    }

    FinishedClusterExtractionJob next_job;
    next_job.current_monot_col_idx = job.current_monot_col_idx;
    next_job.min_monot_azimuth_angle_in_col = min_monot_azimuth_angle_in_col;
    extractFinishedClusters(
        std::move(next_job)); // it has to run in the same thread as both access/modify list of potential cluster roots!
}

void ContinuousClustering::extractFinishedClusters(FinishedClusterExtractionJob&& job)
{
    // keep track of the minimum column index of all unfinished clusters
    int64_t minimum_required_monot_col_idx = std::numeric_limits<int64_t>::max();

    // split into “finished" and "unfinished" clusters
    std::vector<Pixel*> finished_cluster_roots;
    std::vector<Pixel*> unfinished_cluster_roots;

    // iterate over potential cluster roots
    for (Pixel* p : potential_cluster_roots_)
    {
        // discard cluster roots eliminated during union operation
        if (!p->is_potential_cluster_root)
            continue;

        // check whether no more points can be added to this cluster
        if (job.min_monot_azimuth_angle_in_col > p->finished_at_monot_azimuth_angle)
        {
            finished_cluster_roots.push_back(p);
        }
        else
        {
            unfinished_cluster_roots.push_back(p);
            minimum_required_monot_col_idx = std::min(minimum_required_monot_col_idx, p->clust_start_monot_col_idx);
        }
    }

    // replace the old list of potential cluster roots
    potential_cluster_roots_ = unfinished_cluster_roots;

    // if no unfinished clusters, set start index one after current column
    if (minimum_required_monot_col_idx == std::numeric_limits<int64_t>::max())
        minimum_required_monot_col_idx = job.current_monot_col_idx + 1;

    PointCollectionJob next_job;
    next_job.current_monot_col_idx = job.current_monot_col_idx;
    next_job.min_required_monot_col_idx = minimum_required_monot_col_idx;
    next_job.cluster_roots = std::move(finished_cluster_roots);
    point_collection_thread_pool_.enqueue(std::move(next_job));
}

void ContinuousClustering::collectPointsForCusterAndPublish(PointCollectionJob&& job)
{
    // keep track of minimum stamp for this message
    uint64_t min_stamp_for_this_msg = std::numeric_limits<uint64_t>::max();

    // create buffer
    static thread_local std::vector<Pixel> pixels_of_cluster;

    for (Pixel* cluster_root : job.cluster_roots)
    {
        // create cluster id
        int64_t cluster_id = cluster_root->monot_col_idx * num_rows_ + cluster_root->row_idx;

        // collect minimum and maximum stamp for this cluster
        uint64_t min_stamp_for_this_cluster = std::numeric_limits<uint64_t>::max();
        uint64_t max_stamp_for_this_cluster = 0;

        // collect all of its child pixels
        // extension for print after union find
        pixels_of_cluster.clear();
        Pixel* pixel = cluster_root;
        Pixel* start_pixel = pixel;
        if (pixel->stamp_ns < min_stamp_for_this_cluster)
            min_stamp_for_this_cluster = pixel->stamp_ns;
        if (pixel->stamp_ns > max_stamp_for_this_cluster)
            max_stamp_for_this_cluster = pixel->stamp_ns;
        pixel->id = cluster_id;
        pixels_of_cluster.push_back(*pixel);

        while (pixel->next != start_pixel)
        {
            pixel = pixel->next;
            if (pixel->stamp_ns < min_stamp_for_this_cluster)
                min_stamp_for_this_cluster = pixel->stamp_ns;
            if (pixel->stamp_ns > max_stamp_for_this_cluster)
                max_stamp_for_this_cluster = pixel->stamp_ns;
            pixel->id = cluster_id;
            pixels_of_cluster.push_back(*pixel);
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
            finished_cluster_callback_(pixels_of_cluster, stamp_cluster);
        }
    }

    if (finished_column_callback_)
        finished_column_callback_(min_unfinished_monot_col_idx_, job.min_required_monot_col_idx - 1, false);
    min_unfinished_monot_col_idx_ = job.min_required_monot_col_idx;

    // the cols are not cleared here but in the edge generation/association step
}

void ContinuousClustering::clearColumns(int64_t from_monot_col_idx, int64_t to_monot_col_idx)
{
    if (to_monot_col_idx < from_monot_col_idx)
        return;

    for (int64_t monot_col_idx = from_monot_col_idx; monot_col_idx <= to_monot_col_idx; monot_col_idx++)
    {
        int col_idx = static_cast<int>(monot_col_idx % num_columns_);

        for (int row_index = 0; row_index < num_rows_; row_index++)
        {
            // get correct pixel
            Pixel& pixel = range_image_[col_idx * num_rows_ + row_index];

            // raw sensor data
            pixel.xyz.x = std::nanf("");
            pixel.xyz.y = std::nanf("");
            pixel.xyz.z = std::nanf("");
            pixel.firing_idx = 0;
            pixel.intensity = 0;
            pixel.distance = std::nanf("");
            pixel.azimuth_angle = std::nanf("");
            pixel.elevation_angle = std::nanf("");
            pixel.stamp_ns = 0;

            // range image generation
            pixel.col_idx = 0;
            pixel.row_idx = 0;
            pixel.monot_azimuth_angle = std::nan("");
            pixel.monot_col_idx = -1;
            pixel.globally_unique_point_index = static_cast<uint64_t>(-1);

            // ground point segmentation
            pixel.ground_point_label = GP_UNKNOWN;
            pixel.is_ignored = false;
            pixel.height_over_ground = std::nanf("");
            pixel.debug_ground_point_label = WHITE;

            // clustering (union find)
            pixel.parent = nullptr;
            pixel.rank = 0;

            // clustering (infinite cluster detection)
            pixel.clust_start_monot_col_idx = -1;
            pixel.clust_end_monot_col_idx = -1;

            // cluster extraction
            pixel.is_potential_cluster_root = true;
            pixel.finished_at_monot_azimuth_angle = 0.0;
            pixel.next = nullptr;
            pixel.id = 0;

            // debugging
            pixel.number_of_visited_neighbors = 0;
        }
    }
}

void ContinuousClustering::recordJobQueueWorkload(size_t num_jobs_sensor_input)
{
    if (stop_statistics_)
        return;
    num_pending_jobs_.push_back(num_jobs_sensor_input);
    num_pending_jobs_.push_back(range_image_thread_pool_.getNumberOfUnprocessedJobs());
    num_pending_jobs_.push_back(ground_segmentation_thread_pool_.getNumberOfUnprocessedJobs());
    num_pending_jobs_.push_back(union_find_thread_pool_.getNumberOfUnprocessedJobs());
    num_pending_jobs_.push_back(point_collection_thread_pool_.getNumberOfUnprocessedJobs());
    while (num_pending_jobs_.size() > 100000 * 5)
        num_pending_jobs_.pop_front();
}

} // namespace continuous_clustering
