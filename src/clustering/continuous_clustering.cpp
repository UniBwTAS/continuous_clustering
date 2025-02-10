#include <continuous_clustering/clustering/continuous_clustering.hpp>

#include <iostream>
#include <utility>

namespace continuous_clustering
{

ContinuousClustering::ContinuousClustering() = default;

void ContinuousClustering::reset(int num_rows)
{
    // recalculate some intermediate values in case the parameters have changed
    num_columns_ = config_.range_image.num_columns;
    num_rows_ = num_rows;
    srig_azimuth_width_per_column = static_cast<float>((2 * M_PI)) / static_cast<float>(num_columns_);
    ring_buffer_max_columns = num_columns_ * 10;

    // shutdown workers
    insertion_thread_pool.shutdown();
    segmentation_thread_pool.shutdown();
    association_thread_pool.shutdown();
    publishing_thread_pool.shutdown();

    // init/reset range image (implemented as ring buffer)
    range_image_.resize(ring_buffer_max_columns * num_rows);
    clearColumns(0, ring_buffer_max_columns - 1);
    ring_buffer_start_monot_col_idx = -1; // does not start at zero but at the minimum laser of first firing
    ring_buffer_end_monot_col_idx = -1;

    // reset members for continuous range image generation (srig)
    srig_previous_monot_col_idx_of_rearmost_laser = 0;
    srig_previous_monot_col_idx_of_foremost_laser = -1;
    srig_first_unfinished_monot_col_idx = -1;
    reset_required = false;

    // reset members for continuous ground point segmentation (sgps)
    sgps_ego_robot_frame_from_sensor_frame_.reset();

    // reset members for continuous clustering (sc)
    sc_first_unpublished_monot_col_idx = -1;
    sc_cluster_counter_ = 1;
    sc_inclination_angles_between_lasers_.resize(num_rows, std::nanf(""));
    sc_potential_cluster_roots_.clear();

    // re-initialize workers
    int num_treads = config_.general.is_single_threaded ? 0 : 1;
    int num_treads_pub = config_.general.is_single_threaded ? 0 : 1;
    insertion_thread_pool.init(
        [this](InsertionJob&& job) { insertFiringIntoRangeImage(std::forward<InsertionJob>(job)); }, num_treads);
    segmentation_thread_pool.init([this](SegmentationJob&& job)
                                  { performGroundPointSegmentationForColumn(std::forward<SegmentationJob>(job)); },
                                  num_treads);
    association_thread_pool.init(
        [this](AssociationJob&& job) { associatePointsInColumn(std::forward<AssociationJob>(job)); }, num_treads);
    publishing_thread_pool.init([this](PublishingJob&& job)
                                { collectPointsForCusterAndPublish(std::forward<PublishingJob>(job)); },
                                num_treads_pub);
}

void ContinuousClustering::setConfiguration(const Configuration& config)
{
    // some parameter changes need a hard reset
    if (config_.general.is_single_threaded != config.general.is_single_threaded)
        reset_required = true;
    if (config_.range_image.sensor_is_clockwise != config.range_image.sensor_is_clockwise)
        reset_required = true;
    if (config_.range_image.num_columns != config.range_image.num_columns)
        reset_required = true;

    // save new config
    config_ = config;

    // recalculate some values
    max_distance_squared = config_.clustering.max_distance * config_.clustering.max_distance;
}

bool ContinuousClustering::resetRequired() const
{
    return reset_required;
}

void ContinuousClustering::addFiring(const RawPoints::ConstPtr& firing, const Eigen::Isometry3d& odom_from_sensor)
{
    if (num_rows_ != firing->points.size())
        throw std::runtime_error("The number of points in a firing has changed. This is probably a bug!");
    insertion_thread_pool.enqueue({firing, odom_from_sensor});
}

void ContinuousClustering::setFinishedColumnCallback(std::function<void(int64_t, int64_t, bool)> cb)
{
    finished_column_callback_ = std::move(cb);
}

void ContinuousClustering::setFinishedClusterCallback(std::function<void(const std::vector<Point>&, uint64_t)> cb)
{
    finished_cluster_callback_ = std::move(cb);
}

void ContinuousClustering::insertFiringIntoRangeImage(InsertionJob&& job)
{
    // save sensor position in odom frame
    srig_sensor_position = job.odom_frame_from_sensor_frame.translation();

    // sensor position
    sgps_sensor_position.x = static_cast<float>(srig_sensor_position.x());
    sgps_sensor_position.y = static_cast<float>(srig_sensor_position.y());
    sgps_sensor_position.z = static_cast<float>(srig_sensor_position.z());

    // keep track of the global column indices of the foremost and rearmost laser (w.r.t. azimuth angle clockwise =
    // rotation direction of lidar sensor) in this firing
    int64_t monot_col_idx_of_foremost_laser = -1;
    int64_t monot_col_idx_of_rearmost_laser = -1;

    // rotation index
    int64_t previous_rotation_index_of_rearmost_laser =
        srig_previous_monot_col_idx_of_rearmost_laser / num_columns_;

    // process firing from top to bottom
    for (int row_idx = 0; row_idx < job.firing->points.size(); row_idx++)
    {
        // obtain point
        const RawPoint& raw_point = job.firing->points[row_idx];
        Eigen::Vector3d p(raw_point.x, raw_point.y, raw_point.z);

        if (std::isnan(p.x()))
            continue;

        // transform point into odom
        Eigen::Vector3d p_odom = job.odom_frame_from_sensor_frame * p;

        // get point relative to sensor origin
        Eigen::Vector3d p_odom_rel = p_odom - srig_sensor_position;

        // calculate azimuth angle
        // float azimuth_angle = std::atan2(static_cast<float>(p_odom_rel.y()), static_cast<float>(p_odom_rel.x()));
        float azimuth_angle = std::atan2(static_cast<float>(p.y()), static_cast<float>(p.x()));

        // calculate azimuth angle which starts at negative X-axis with 0 and increases with ongoing lidar rotation
        // to 2 pi, which is more intuitive and important for fast array index calculation
        float increasing_azimuth_angle = config_.range_image.sensor_is_clockwise ?
                                             -azimuth_angle + static_cast<float>(M_PI) :
                                             azimuth_angle + static_cast<float>(M_PI);

        // global column index
        int column_index_within_rotation = static_cast<int>(increasing_azimuth_angle / srig_azimuth_width_per_column);
        int64_t monot_col_idx =
            previous_rotation_index_of_rearmost_laser * num_columns_ + column_index_within_rotation;

        // check if we hit negative x-axis with w.r.t. previous firing
        int column_index_within_rotation_of_previous_rearmost_laser =
            static_cast<int>(srig_previous_monot_col_idx_of_rearmost_laser % num_columns_);
        int column_diff = column_index_within_rotation - column_index_within_rotation_of_previous_rearmost_laser;
        int cols_of_half_rotation = num_columns_ / 2;
        int rotation_index_offset = 0;
        if (column_diff < -cols_of_half_rotation)
        {
            monot_col_idx += num_columns_; // add one rotation
            rotation_index_offset = 1;
        }
        else if (srig_previous_monot_col_idx_of_rearmost_laser > 0 && column_diff > cols_of_half_rotation)
        {
            // In very rare cases this can happen because the minimum azimuth of rearmost laser can be smaller than
            // that of previous firing (most probably due to ego motion correction).
            // This gets only tricky when srig_previous_monot_col_idx_of_rearmost_laser % num_columns == 0 and
            // azimuth of rearmost laser in current firing is smaller than previous one.
            // So we have to subtract one rotation.
            monot_col_idx -= num_columns_; // subtract one rotation
            rotation_index_offset = -1;
        }

        // local column index
        int col_idx = static_cast<int>(monot_col_idx % ring_buffer_max_columns);

        // get correct point
        Point* point = &range_image_[col_idx * num_rows_ + row_idx]; // column major order

        // calculate continuous azimuth angle (even if we move it to the next cell, this value remains the same)
        double monot_azimuth_angle =
            (2 * M_PI) * static_cast<double>(previous_rotation_index_of_rearmost_laser + rotation_index_offset) +
            increasing_azimuth_angle;

        // in case this cell is already occupied, try next column
        auto distance = static_cast<float>(p_odom_rel.norm());
        if (!std::isnan(point->distance) && !std::isnan(distance))
        {
            int next_col_idx = col_idx + 1;
            if (next_col_idx >= ring_buffer_max_columns)
                next_col_idx -= ring_buffer_max_columns;
            Point* next_point = &range_image_[next_col_idx * num_rows_ + row_idx];
            if (std::isnan(next_point->distance))
            {
                point = next_point;
                col_idx = next_col_idx;
                monot_col_idx++;
            }
        }

        // avoid that a valid cell (non-nan) is overwritten by a nan or more distant value
        if (!std::isnan(point->distance) && (std::isnan(distance) || distance >= point->distance))
            continue;

        // do not insert into cols that were passed to the next processing step
        bool laser_too_far_behind = false;
        if (srig_first_unfinished_monot_col_idx >= 0 &&
            monot_col_idx < srig_first_unfinished_monot_col_idx)
        {
            /*ROS_WARN_STREAM("Ignore point of firing because it would be inserted into an already published column. "
                            "Wanted to insert at "
                            << monot_col_idx << ", but first unfinished global column index is already at "
                            << srig_first_unfinished_monot_col_idx << " (row index: " << row_index << ")");*/

            laser_too_far_behind = true;
        }

        // fill point data
        if (!laser_too_far_behind)
        {
            point->xyz.x = static_cast<float>(p_odom.x());
            point->xyz.y = static_cast<float>(p_odom.y());
            point->xyz.z = static_cast<float>(p_odom.z());
            point->firing_idx = raw_point.firing_index;
            point->intensity = raw_point.intensity;
            point->stamp_ns = raw_point.stamp;
            point->distance = distance;
            point->azimuth_angle = azimuth_angle;
            point->elevation_angle = std::asin(static_cast<float>(p_odom_rel.z()) / point->distance);
            point->monot_azimuth_angle = monot_azimuth_angle; // omitted cells will be filled again later
            point->col_idx = col_idx;
            point->row_idx = row_idx;
            point->monot_col_idx = monot_col_idx;            // omitted cells will be filled again later
            point->globally_unique_point_index = raw_point.globally_unique_point_index;
        }

        // keep track of global column index of rearmost & foremost laser
        if (monot_col_idx_of_rearmost_laser < 0 || monot_col_idx < monot_col_idx_of_rearmost_laser)
            monot_col_idx_of_rearmost_laser = monot_col_idx;
        if (monot_col_idx_of_foremost_laser < 0 || monot_col_idx > monot_col_idx_of_foremost_laser)
            monot_col_idx_of_foremost_laser = monot_col_idx;
    }

    // if for this firing a minimum/maximum column index was found then use it as the new one
    if (monot_col_idx_of_rearmost_laser >= 0 && monot_col_idx_of_foremost_laser >= 0)
    {
        // if the azimuth range of the firing covers more than 180 degrees, this means that the firing is
        // intersected with negative x-axis (this means that the range image was incorrectly filled -> reset)
        if ((monot_col_idx_of_foremost_laser - monot_col_idx_of_rearmost_laser) > num_columns_ / 2)
        {
            std::cout << "Very first firing after reset intersects with negative x-axis: " +
                             std::to_string(monot_col_idx_of_rearmost_laser) + ", " +
                             std::to_string(monot_col_idx_of_foremost_laser) + ", " +
                             std::to_string(srig_previous_monot_col_idx_of_rearmost_laser) +
                             ". This is invalid. Reset continuous clustering on next message.";
            reset_required = true;
            return;
        }

        if (monot_col_idx_of_rearmost_laser > srig_previous_monot_col_idx_of_rearmost_laser)
            srig_previous_monot_col_idx_of_rearmost_laser = monot_col_idx_of_rearmost_laser;
        if (monot_col_idx_of_foremost_laser > srig_previous_monot_col_idx_of_foremost_laser)
            srig_previous_monot_col_idx_of_foremost_laser = monot_col_idx_of_foremost_laser;
    }

    // there is no information about minimum and maximum global column index
    if (srig_previous_monot_col_idx_of_foremost_laser < 0)
        return;

    // initialize start of ring buffer
    if (ring_buffer_start_monot_col_idx == -1)
    {
        ring_buffer_start_monot_col_idx = srig_previous_monot_col_idx_of_rearmost_laser;
        sc_first_unpublished_monot_col_idx = srig_previous_monot_col_idx_of_rearmost_laser;
    }

    // update end of ring buffer (maximum global column index ever seen)
    if (srig_previous_monot_col_idx_of_foremost_laser > ring_buffer_end_monot_col_idx)
        ring_buffer_end_monot_col_idx = srig_previous_monot_col_idx_of_foremost_laser;

    // start publishing at index of last laser of very first firing
    if (srig_first_unfinished_monot_col_idx == -1)
        srig_first_unfinished_monot_col_idx = srig_previous_monot_col_idx_of_rearmost_laser;

    // iterate over finished but unfinished cols and publish them
    while (srig_first_unfinished_monot_col_idx < srig_previous_monot_col_idx_of_rearmost_laser)
        segmentation_thread_pool.enqueue(
            {srig_first_unfinished_monot_col_idx++, job.odom_frame_from_sensor_frame});
}

void ContinuousClustering::performGroundPointSegmentationForColumn(SegmentationJob&& job)
{
    int col_idx = static_cast<int>(job.ring_buffer_current_monot_col_idx % ring_buffer_max_columns);

    if (!sgps_ego_robot_frame_from_sensor_frame_)
        throw std::runtime_error("Transform robot frame from sensor frame was not set yet!");
    Eigen::Isometry3d ego_robot_frame_from_odom_frame =
        *sgps_ego_robot_frame_from_sensor_frame_ * job.odom_frame_from_sensor_frame.inverse();
    float height_sensor_to_ground = -static_cast<float>(sgps_ego_robot_frame_from_sensor_frame_->translation().z()) +
                                    config_.ground_segmentation.height_ref_to_ground_;

    // iterate rows from bottom to top and find ground points
    bool first_obstacle_detected = false;
    bool first_point_found = false;
    Point3D last_ground_position_wrt_sensor{0, 0, height_sensor_to_ground};
    Point3D ground_x_meter_behind_position_wrt_sensor = {0, 0, 0};
    Point3D previous_position_wrt_sensor;
    uint8_t previous_label;
    float inclination_previous_laser = 0; // calculate difference between elevation angles for subsequent steps

    for (int row_index = num_rows_ - 1; row_index >= 0; row_index--)
    {
        // obtain point
        Point& point = range_image_[col_idx * num_rows_ + row_index];

        // check if there is a problem with the ring buffer
        int64_t point_monot_col_idx_copy = point.monot_col_idx;
        if (point_monot_col_idx_copy != job.ring_buffer_current_monot_col_idx &&
            point_monot_col_idx_copy != -1)
        {
            stop_statistics = true;
            /*std::string filename = std::tmpnam(nullptr);
            std::cout << "JOB QUEUES (INSERT, SEGMENT, ASSOC, PUB): "
                      << insertion_thread_pool.getNumberOfUnprocessedJobs() << ", "
                      << segmentation_thread_pool.getNumberOfUnprocessedJobs() << ", "
                      << association_thread_pool.getNumberOfUnprocessedJobs() << ", "
                      << publishing_thread_pool.getNumberOfUnprocessedJobs() << std::endl;
            std::cout << "Writing statistics to: " << filename << std::endl;
            std::ofstream out(filename);
            for (auto n : num_pending_jobs)
                out << n << ", ";
            out.close();*/
            throw std::runtime_error(
                "This column is not cleared. Probably this means the ring buffer is full or there "
                "is some other issue with clearing (not cleared at all or written after clearing): " +
                std::to_string(point_monot_col_idx_copy) + ", " +
                std::to_string(job.ring_buffer_current_monot_col_idx) + ", " +
                std::to_string(ring_buffer_max_columns) +
                "; This typically happens when the clustering is not fast enough to handle all the firings. Consider "
                "to play the sensor data more slowly or to adjust the parameters to make the clustering faster.");
        }

        // refill local/global column index because it was not filled for omitted cells
        point.monot_col_idx = job.ring_buffer_current_monot_col_idx;
        point.col_idx = static_cast<int>(job.ring_buffer_current_monot_col_idx % ring_buffer_max_columns);

        // keep track of (differences between) the elevation angles of the lasers (for later processing steps)
        float inclination_current_laser = range_image_[col_idx * num_rows_ + row_index].elevation_angle;
        float diff = inclination_current_laser - inclination_previous_laser;
        if (!std::isnan(diff))
            sc_inclination_angles_between_lasers_[row_index] = diff; // last value is useless but we do not use it
        inclination_previous_laser = inclination_current_laser;

        // skip NaN's
        if (std::isnan(point.distance))
        {
            // use elevation angle from previous column (it is useful to supplement nan cells with an elevation
            // angle in order to be able to break the while loop earlier during association
            if (config_.range_image.supplement_inclination_angle_for_nan_cells && row_index < num_rows_ - 1)
            {
                Point& point_below = range_image_[col_idx * num_rows_ + (row_index + 1)];
                point.elevation_angle =
                    point_below.elevation_angle + sc_inclination_angles_between_lasers_[row_index];
            }
            // recalculate continuous azimuth for omitted/NaN cells (for later processing steps)
            point.monot_azimuth_angle = (static_cast<double>(job.ring_buffer_current_monot_col_idx) + 0.5) *
                                             srig_azimuth_width_per_column;
            continue;
        }

        // skip points which seem to be fog
        if (config_.ground_segmentation.fog_filtering_enabled &&
            point.intensity < config_.ground_segmentation.fog_filtering_intensity_below &&
            point.distance < config_.ground_segmentation.fog_filtering_distance_below &&
            point.elevation_angle > config_.ground_segmentation.fog_filtering_inclination_above)
        {
            point.ground_point_label = GP_FOG;
            point.debug_ground_point_label = LIGHTGRAY;
            continue;
        }

        const Point3D& current_position = point.xyz;

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
            point.ground_point_label = GP_EGO_VEHICLE;
            point.debug_ground_point_label = VIOLET;
            continue;
        }

        Point3D current_position_wrt_sensor = current_position - sgps_sensor_position;

        // special handling first point outside the ego bounding box
        if (!first_point_found)
        {
            // now we found the first point outside the ego vehicle box
            first_point_found = true;
            float height_over_predicted_ground = current_position_wrt_sensor.z - height_sensor_to_ground;
            if (height_over_predicted_ground > c.first_ring_as_ground_min_allowed_z_diff &&
                height_over_predicted_ground < c.first_ring_as_ground_max_allowed_z_diff)
            {
                point.ground_point_label = GP_GROUND;
                point.debug_ground_point_label = GRAY;
                last_ground_position_wrt_sensor = current_position_wrt_sensor;
                first_obstacle_detected = false;
            }
            else
            {
                point.ground_point_label = GP_OBSTACLE;
                point.debug_ground_point_label = ORANGE;
                first_obstacle_detected = true;
            }
            previous_position_wrt_sensor = current_position_wrt_sensor;
            previous_label = point.debug_ground_point_label;
            continue;
        }

        // calculate the slope w.r.t previous point
        Point2D current_position_wrt_sensor_2d = to2dInAzimuthPlane(current_position_wrt_sensor);
        Point2D previous_position_wrt_sensor_2d = to2dInAzimuthPlane(previous_position_wrt_sensor);
        Point2D previous_to_current = current_position_wrt_sensor_2d - previous_position_wrt_sensor_2d;
        float slope_to_prev = previous_to_current.y / previous_to_current.x;
        bool is_flat_wrt_prev = std::abs(slope_to_prev) < c.max_slope && previous_to_current.x > 0;
        is_flat_wrt_prev = is_flat_wrt_prev && (!c.use_terrain || previous_to_current.x < 5); // TODO: Magic number

        // calculate slope w.r.t. last seen (quite certain) ground point
        Point2D last_ground_position_wrt_sensor_2d = to2dInAzimuthPlane(last_ground_position_wrt_sensor);
        Point2D last_ground_to_current = current_position_wrt_sensor_2d - last_ground_position_wrt_sensor_2d;
        float slope_to_last_ground = last_ground_to_current.y / last_ground_to_current.x;
        bool is_flat_wrt_last_ground = std::abs(slope_to_last_ground) < c.max_slope && last_ground_to_current.x > 0;

        // quite certain ground points
        if (!first_obstacle_detected && is_flat_wrt_prev)
        {
            point.ground_point_label = GP_GROUND;
            point.debug_ground_point_label = GREEN;
        }
        else // try to find remaining ground points
        {
            if (c.use_terrain)
            {
                /*if (last_terrain_msg_)
                {
                    auto& info = last_terrain_msg_->info;
                    Point3D terrain_min_corner(static_cast<float>(info.pose.position.x - info.length_x / 2),
                                               static_cast<float>(info.pose.position.y - info.length_y / 2),
                                               static_cast<float>(info.pose.position.z));
                    Point3D current_position_in_terrain = current_position - terrain_min_corner;
                    auto& data = last_terrain_msg_->data[0];
                    int num_cells_x = static_cast<int>(data.layout.dim[0].size);
                    int num_cells_y = static_cast<int>(data.layout.dim[1].size);
                    int idx_x = num_cells_x -
                                static_cast<int>(
                                    std::floor(current_position_in_terrain.x / static_cast<float>(info.resolution))) -
                                1;
                    int idx_y = num_cells_y -
                                static_cast<int>(
                                    std::floor(current_position_in_terrain.y / static_cast<float>(info.resolution))) -
                                1;
                    if (idx_x >= 0 && idx_x < num_cells_x && idx_y >= 0 && idx_y < num_cells_y)
                    {
                        uint32_t data_idx = idx_y * num_cells_x + idx_x;
                        if (data_idx >= 0 && data_idx < data.data.size())
                        {
                            float relative_height = current_position.z - data.data[data_idx];
                            if (std::abs(relative_height) < c.terrain_max_allowed_z_diff)
                            {
                                point.ground_point_label = GP_GROUND;
                                point.debug_ground_point_label = BURLYWOOD;
                            }
                        }
                    }
                }*/
            }
            else
            {
                if (first_obstacle_detected && is_flat_wrt_prev && is_flat_wrt_last_ground)
                {
                    point.ground_point_label = GP_GROUND;
                    point.debug_ground_point_label = YELLOWGREEN;
                }
                else if (std::abs(last_ground_to_current.x) <
                             c.ground_because_close_to_last_certain_ground_max_dist_diff &&
                         std::abs(last_ground_to_current.y) < c.ground_because_close_to_last_certain_ground_max_z_diff)
                {
                    point.ground_point_label = GP_GROUND;
                    point.debug_ground_point_label = YELLOW;
                }
            }
        }

        // mark remaining points as obstacle
        if (point.ground_point_label != GP_GROUND)
        {
            point.ground_point_label = GP_OBSTACLE;
            point.debug_ground_point_label = RED;

            // go down in the rows and mark very close points also as obstacle
            int prev_row_index = row_index + 1;
            while (prev_row_index < num_rows_)
            {
                Point& cur_point = range_image_[col_idx * num_rows_ + prev_row_index];
                Point2D prev_position_wrt_sensor_2d = to2dInAzimuthPlane(cur_point.xyz - sgps_sensor_position);
                if (cur_point.debug_ground_point_label == YELLOW ||
                    (cur_point.ground_point_label == GP_GROUND &&
                     std::abs((current_position_wrt_sensor_2d - prev_position_wrt_sensor_2d).x) <
                         c.obstacle_because_next_certain_obstacle_max_dist_diff))
                {
                    if (cur_point.ground_point_label == GP_GROUND)
                    {
                        cur_point.ground_point_label = GP_OBSTACLE;
                        cur_point.debug_ground_point_label = DARKRED;
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
        first_obstacle_detected |= point.ground_point_label == GP_OBSTACLE;

        // keep track of last (certain) ground point
        if (point.debug_ground_point_label == GREEN || point.debug_ground_point_label == YELLOWGREEN)
        {
            // only use current point as the new last ground point when it was plausible. On wet streets there are often
            // false points below the ground surface because of reflections. Therefore, we do not want the slope to be
            // too much going down. Furthermore, in this case often there is a larger distance jump.
            if (slope_to_prev > c.last_ground_point_slope_higher_than &&
                std::abs(previous_to_current.x) < c.last_ground_point_distance_smaller_than && previous_label != YELLOW)
            {
                last_ground_position_wrt_sensor = current_position_wrt_sensor;
            }
            /*else if (previous_label == YELLOW)
            {
                point.debug_ground_point_label = CYAN;
            }
            else
            {
                point.debug_ground_point_label = BLACK;
            }*/
        }

        // keep track of previous point
        previous_position_wrt_sensor = current_position_wrt_sensor;
        previous_label = point.debug_ground_point_label;
    }

    for (int row_index = num_rows_ - 1; row_index >= 0; row_index--)
    {
        int current_data_index_ri = col_idx * num_rows_ + row_index; // column major
        Point& point = range_image_[current_data_index_ri];

        // prepare everything for next step in pipeline (point association)
        point.is_ignored = false;

        // ignore this point if it is NaN
        if (std::isnan(point.distance))
        {
            point.is_ignored = true;
            continue;
        }

        // only consider obstacle points
        if (point.ground_point_label != GP_OBSTACLE)
        {
            point.is_ignored = true;
            continue;
        }

        // ignore this point if it is too close
        if (point.distance < 1. * config_.clustering.max_distance)
        {
            point.is_ignored = true;
            continue;
        }

        // ignore this point if the distance in combination with elevation diff can't be below distance threshold
        if (config_.clustering.ignore_points_with_too_big_inclination_angle_diff && row_index < (num_rows_ - 1) &&
            std::atan2(config_.clustering.max_distance, point.distance) <
                sc_inclination_angles_between_lasers_[row_index])
        {
            point.is_ignored = true;
            continue;
        }

        // ignore this points in a chessboard pattern
        if (config_.clustering.ignore_points_in_chessboard_pattern)
        {
            bool column_even = point.monot_col_idx % 2 == 0;
            bool row_even = row_index % 2 == 0;
            if ((column_even && !row_even) || (!column_even && row_even))
            {
                point.is_ignored = true;
                continue;
            }
        }
    }

    if (finished_column_callback_)
        finished_column_callback_(
            job.ring_buffer_current_monot_col_idx, job.ring_buffer_current_monot_col_idx, true);

    // lets enqueue the association job for this column to do it in a separate thread
    association_thread_pool.enqueue({job.ring_buffer_current_monot_col_idx});
}

void ContinuousClustering::setTransformRobotFrameFromSensorFrame(const Eigen::Isometry3d& tf)
{
    if (!sgps_ego_robot_frame_from_sensor_frame_)
        sgps_ego_robot_frame_from_sensor_frame_ = std::make_unique<Eigen::Isometry3d>();
    *sgps_ego_robot_frame_from_sensor_frame_ = tf;
}

bool ContinuousClustering::hasTransformRobotFrameFromSensorFrame()
{
    return sgps_ego_robot_frame_from_sensor_frame_ != nullptr;
}

bool ContinuousClustering::checkClusteringCondition(const Point& point, const Point& point_other) const
{
    return (point.xyz - point_other.xyz).lengthSquared() < max_distance_squared;
}

void ContinuousClustering::make_set(Point* point, float max_angle_diff)
{
    // regular union find algorithm
    point->parent = point;

    // extension for print after union find
    point->next = point;

    // extension for range image meta data
    point->finished_at_monot_azimuth_angle = point->monot_azimuth_angle + max_angle_diff;
    point->clust_start_monot_col_idx = point->monot_col_idx;
    point->cluster_end_monot_col_idx = point->monot_col_idx;
    point->is_potential_cluster_root = false;
}

Point* ContinuousClustering::find_set(Point* point)
{
    // regular union find algorithm with path compression

    // find root vertex of current tree
    Point* root = point;
    while (root != root->parent)
        root = root->parent;

    // make each vertex in path direct child of root vertex
    while (point != point->parent)
    {
        Point* next_vertex = point->parent;
        point->parent = root;
        point = next_vertex;
    }

    return root;
}

void ContinuousClustering::link_set(Point* point, Point* point_other)
{
    // regular union find algorithm (with union by rank)
    Point* root_after_union;
    Point* child_after_union;
    if (point->rank > point_other->rank)
    {
        point_other->parent = point;
        root_after_union = point;
        child_after_union = point_other;
    }
    else
    {
        point->parent = point_other;
        if (point->rank == point_other->rank)
            point_other->rank++;
        root_after_union = point_other;
        child_after_union = point;
    }

    // extension for print after union find
    Point* tmp = point_other->next;
    point_other->next = point->next;
    point->next = tmp;

    // extension for range image meta data
    root_after_union->finished_at_monot_azimuth_angle =
        std::max(point->finished_at_monot_azimuth_angle, point_other->finished_at_monot_azimuth_angle);
    root_after_union->clust_start_monot_col_idx =
        std::min(point->clust_start_monot_col_idx, point_other->clust_start_monot_col_idx);
    root_after_union->cluster_end_monot_col_idx =
        std::max(point->cluster_end_monot_col_idx, point_other->cluster_end_monot_col_idx);
    child_after_union->is_potential_cluster_root = false;
}

bool ContinuousClustering::union_set(Point* point, Point* point_other)
{
    // regular union find algorithm
    Point* root_of_point = find_set(point);
    Point* root_of_point_other = find_set(point_other);

    // extension for infinite cluster detection & avoidance
    uint16_t num_columns =
        root_of_point->cluster_end_monot_col_idx - root_of_point->clust_start_monot_col_idx + 1;
    uint16_t num_columns_other = root_of_point_other->cluster_end_monot_col_idx -
                                 root_of_point_other->clust_start_monot_col_idx + 1;
    if (num_columns >= num_columns_ || num_columns_other >= num_columns_)
        return false;

    // regular union find algorithm
    if (root_of_point != root_of_point_other)
        link_set(root_of_point, root_of_point_other);

    return true;
}

void ContinuousClustering::print_set(Point* point, std::vector<Point>& v)
{
    // extension for print after union find
    v.clear();
    Point* starting_point = point;
    v.push_back(*point);
    while (point->next != starting_point)
    {
        point = point->next;
        v.push_back(*point);
    }
}

bool ContinuousClustering::traverseFieldOfView(Point& point,
                                               float max_angle_diff,
                                               int ring_buffer_first_col_idx)
{
    // go left each column until azimuth angle difference gets too large
    bool at_least_one_edge = false;
    int required_steps_back = static_cast<int>(std::ceil(max_angle_diff / srig_azimuth_width_per_column));
    required_steps_back = std::min(required_steps_back, config_.clustering.max_steps_in_row);
    int64_t other_col_idx = point.col_idx;
    for (int num_steps_back = 0; num_steps_back <= required_steps_back; num_steps_back++)
    {
        for (int direction = -1; direction <= 1; direction += 2)
        {
            // do not go down in first column (these points are not associated to tree yet!)
            if (direction == 1 && num_steps_back == 0)
                continue;

            // go up/down each row until the elevation angle difference gets too large
            int num_steps_vertical = direction == 1 || num_steps_back == 0 ? 1 : 0;
            int other_row_index =
                direction == 1 || num_steps_back == 0 ? point.row_idx + direction : point.row_idx;
            while (other_row_index >= 0 && other_row_index < num_rows_ &&
                   num_steps_vertical <= config_.clustering.max_steps_in_column)
            {
                // get other point
                Point& point_other = range_image_[other_col_idx * num_rows_ + other_row_index];

                // count number of visited points for analyzing
                point.number_of_visited_neighbors += 1;

                // no cluster can be associated because the elevation angle diff gets too large
                if (std::abs(point_other.elevation_angle - point.elevation_angle) > max_angle_diff)
                    break;

                // if other point is ignored or has already the same tree root then do nothing (*1)
                if (!point_other.is_ignored &&
                    checkClusteringCondition(point, point_other)) // TODO: UF make find before clustering condition?
                {
                    at_least_one_edge = union_set(&point, &point_other);
                }

                // stop searching if point was already associated and minimum number of cols were processed
                if (point.parent != &point && config_.clustering.stop_after_association_enabled &&
                    num_steps_vertical >= config_.clustering.stop_after_association_min_steps)
                    break;

                other_row_index += direction;
                num_steps_vertical++;
            }
        }

        // stop searching if point was already associated and minimum number of points were processed
        if (point.parent != &point && config_.clustering.stop_after_association_enabled &&
            num_steps_back >= config_.clustering.stop_after_association_min_steps)
            break;

        // stop searching if we are at the beginning of the ring buffer
        if (other_col_idx == ring_buffer_first_col_idx)
            break;

        other_col_idx--;

        // jump to the end of the ring buffer
        if (other_col_idx < 0)
            other_col_idx += ring_buffer_max_columns;
    }

    return at_least_one_edge;
}

void ContinuousClustering::associatePointsInColumn(AssociationJob&& job)
{
    // clear all cols that are not needed anymore
    int64_t prev_ring_buffer_start_monot_col_idx = ring_buffer_start_monot_col_idx;
    ring_buffer_start_monot_col_idx = sc_first_unpublished_monot_col_idx;
    clearColumns(prev_ring_buffer_start_monot_col_idx, ring_buffer_start_monot_col_idx - 1);

    // keep track of the current minimum azimuth angle of the current column
    double current_minimum_monot_azimuth_angle = std::numeric_limits<double>::max();

    // get local start index of ring buffer start
    int ring_buffer_first_col_idx =
        static_cast<int>(sc_first_unpublished_monot_col_idx % ring_buffer_max_columns);

    // get current local index of ring buffer start
    int ring_buffer_current_col_idx =
        static_cast<int>(job.ring_buffer_current_monot_col_idx % ring_buffer_max_columns);

    for (int row_index = 0; row_index < num_rows_; row_index++)
    {
        // get current point
        Point& point = range_image_[ring_buffer_current_col_idx * num_rows_ + row_index];

        // keep track of the current minimum continuous azimuth angle
        if (point.monot_azimuth_angle < current_minimum_monot_azimuth_angle)
            current_minimum_monot_azimuth_angle = point.monot_azimuth_angle;

        // check whether point should be ignored
        if (point.is_ignored)
            continue;

        // calculate minimum possible azimuth angle
        float max_angle_diff = std::asin(config_.clustering.max_distance / point.distance);

        // initialize a new cluster containing only this point (initialize for union find)
        make_set(&point, max_angle_diff);

        // traverse field of view
        bool neighbor_found = traverseFieldOfView(point, max_angle_diff, ring_buffer_first_col_idx);
        if (!neighbor_found)
        {
            point.is_potential_cluster_root = true;
            sc_potential_cluster_roots_.push_back(&point);
        } else {
            point.is_potential_cluster_root = false;
        }
    }

    int64_t minimum_required_monot_col_idx = std::numeric_limits<int64_t>::max();
    std::list<Point*> finished_cluster_roots;
    auto it = sc_potential_cluster_roots_.begin();
    while (it != sc_potential_cluster_roots_.end())
    {
        Point* potential_cluster_root = *it;

        bool laser_diodes_far_enough =
            current_minimum_monot_azimuth_angle > potential_cluster_root->finished_at_monot_azimuth_angle;
        bool root_eliminated_by_union_set = !potential_cluster_root->is_potential_cluster_root;

        if (laser_diodes_far_enough && !root_eliminated_by_union_set)
            finished_cluster_roots.push_back(*it);

        if (root_eliminated_by_union_set || laser_diodes_far_enough)
        {
            it = sc_potential_cluster_roots_.erase(it);
        }
        else
        {
            ++it;
            if (potential_cluster_root->clust_start_monot_col_idx < minimum_required_monot_col_idx)
                minimum_required_monot_col_idx = potential_cluster_root->clust_start_monot_col_idx;
        }
    }

    // if there are no unfinished clusters then set the start index one after current global column index
    if (minimum_required_monot_col_idx == std::numeric_limits<int64_t>::max())
        minimum_required_monot_col_idx = job.ring_buffer_current_monot_col_idx + 1;

    PublishingJob next_job;
    next_job.ring_buffer_current_monot_col_idx = job.ring_buffer_current_monot_col_idx;
    next_job.ring_buffer_min_required_monot_col_idx = minimum_required_monot_col_idx;
    next_job.cluster_roots = std::move(finished_cluster_roots);
    publishing_thread_pool.enqueue(std::move(next_job));
}

void ContinuousClustering::collectPointsForCusterAndPublish(PublishingJob&& job)
{
    // keep track of minimum stamp for this message
    uint64_t min_stamp_for_this_msg = std::numeric_limits<uint64_t>::max();

    // create buffer
    static thread_local std::vector<Point> cluster_points;

    for (Point* cluster_root : job.cluster_roots)
    {
        // create cluster id
        int64_t cluster_id = cluster_root->monot_col_idx * num_rows_ + cluster_root->row_idx;

        // collect minimum and maximum stamp for this cluster
        uint64_t min_stamp_for_this_cluster = std::numeric_limits<uint64_t>::max();
        uint64_t max_stamp_for_this_cluster = 0;

        // collect all of its child points
        // extension for print after union find
        cluster_points.clear();
        Point* point = cluster_root;
        Point* starting_point = point;
        if (point->stamp_ns < min_stamp_for_this_cluster)
            min_stamp_for_this_cluster = point->stamp_ns;
        if (point->stamp_ns > max_stamp_for_this_cluster)
            max_stamp_for_this_cluster = point->stamp_ns;
        point->id = cluster_id;
        cluster_points.push_back(*point);

        while (point->next != starting_point)
        {
            point = point->next;
            if (point->stamp_ns < min_stamp_for_this_cluster)
                min_stamp_for_this_cluster = point->stamp_ns;
            if (point->stamp_ns > max_stamp_for_this_cluster)
                max_stamp_for_this_cluster = point->stamp_ns;
            point->id = cluster_id;
            cluster_points.push_back(*point);
        }

        // keep track of minimum stamp for this message
        if (min_stamp_for_this_cluster < min_stamp_for_this_msg)
            min_stamp_for_this_msg = min_stamp_for_this_cluster;

        // publish points (TODO: make threshold configurable)
        if (cluster_points.size() > 20 && finished_cluster_callback_)
        {
            uint64_t stamp_cluster =
                config_.clustering.use_last_point_for_cluster_stamp ?
                    max_stamp_for_this_cluster :
                    min_stamp_for_this_cluster + (max_stamp_for_this_cluster - min_stamp_for_this_cluster) / 2;
            finished_cluster_callback_(cluster_points, stamp_cluster);
        }
    }

    if (finished_column_callback_)
        finished_column_callback_(
            sc_first_unpublished_monot_col_idx, job.ring_buffer_min_required_monot_col_idx - 1, false);
    sc_first_unpublished_monot_col_idx = job.ring_buffer_min_required_monot_col_idx;

    // the cols are not cleared here but in the edge generation/association step 
}

void ContinuousClustering::clearColumns(int64_t from_monot_col_idx, int64_t to_monot_col_idx)
{
    if (to_monot_col_idx < from_monot_col_idx)
        return;

    for (int64_t monot_col_idx = from_monot_col_idx; monot_col_idx <= to_monot_col_idx;
         monot_col_idx++)
    {
        int col_idx = static_cast<int>(monot_col_idx % ring_buffer_max_columns);

        for (int row_index = 0; row_index < num_rows_; row_index++)
        {
            // get correct point
            Point& point = range_image_[col_idx * num_rows_ + row_index];

            // raw sensor data
            point.xyz.x = std::nanf("");
            point.xyz.y = std::nanf("");
            point.xyz.z = std::nanf("");
            point.firing_idx = 0;
            point.intensity = 0;
            point.distance = std::nanf("");
            point.azimuth_angle = std::nanf("");
            point.elevation_angle = std::nanf("");
            point.stamp_ns = 0;

            // range image generation
            point.col_idx = 0;
            point.row_idx = 0;
            point.monot_azimuth_angle = std::nan("");
            point.monot_col_idx = -1;            
            point.globally_unique_point_index = static_cast<uint64_t>(-1);

            // ground point segmentation
            point.ground_point_label = GP_UNKNOWN;
            point.is_ignored = false;
            point.height_over_ground = std::nanf("");
            point.debug_ground_point_label = WHITE;

            // clustering (union find)
            point.parent = nullptr;
            point.rank = 0;

            // clustering (infinite cluster detection)
            point.clust_start_monot_col_idx = -1;
            point.cluster_end_monot_col_idx = -1;

            // cluster extraction
            point.is_potential_cluster_root = true;
            point.finished_at_monot_azimuth_angle = 0.0;
            point.next = nullptr;
            point.id = 0;

            // debugging
            point.number_of_visited_neighbors = 0;
        }
    }
}

void ContinuousClustering::recordJobQueueWorkload(size_t num_jobs_sensor_input)
{
    if (stop_statistics)
        return;
    num_pending_jobs.push_back(num_jobs_sensor_input);
    num_pending_jobs.push_back(insertion_thread_pool.getNumberOfUnprocessedJobs());
    num_pending_jobs.push_back(segmentation_thread_pool.getNumberOfUnprocessedJobs());
    num_pending_jobs.push_back(association_thread_pool.getNumberOfUnprocessedJobs());
    num_pending_jobs.push_back(publishing_thread_pool.getNumberOfUnprocessedJobs());
    while (num_pending_jobs.size() > 100000 * 5)
        num_pending_jobs.pop_front();
}

} // namespace continuous_clustering
