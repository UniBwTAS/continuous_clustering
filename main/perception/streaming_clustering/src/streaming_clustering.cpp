#include <streaming_clustering/velodyne_input/velodyne_input.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <streaming_clustering/streaming_clustering.h>

#include <memory>

namespace streaming_clustering
{

StreamingClustering::StreamingClustering(ros::NodeHandle nh, const ros::NodeHandle& nh_private)
{
    // obtain parameters
    nh_private.param<std::string>("sensor_name", sensor_name, "vls_128");
    nh_private.param<std::string>("sensor_frame", sensor_frame, "sensor/lidar/vls128_roof");
    nh_private.param<std::string>("odom_frame", odom_frame, "odom");
    nh_private.param<int>("columns_per_full_rotation", num_columns, 1877);
    nh_private.param<bool>("wait_for_tf", wait_for_tf, true);
    nh_private.param<bool>(
        "supplement_inclination_angle_for_nan_cells", sc_supplement_inclination_angle_for_nan_cells, true);
    nh_private.param<bool>("use_last_point_for_cluster_stamp", sc_use_last_point_for_cluster_stamp, false);
    srig_azimuth_width_per_column = static_cast<float>((2 * M_PI)) / static_cast<float>(num_columns);
    ring_buffer_max_columns = num_columns * 10;

    // use desired sensor input
    if (sensor_name == "vls_128")
        sensor_input_.reset(new VelodyneInput(nh, nh_private));
    else
        throw std::runtime_error("Unknown sensor: " + sensor_name);
    sensor_input_->subscribe();
    sensor_input_->addOnNewFiringCallback([this](const RawPoints::ConstPtr& firing) { onNewFiringsCallback(firing); });

    // obtain ego vehicle parameters (important to remove points on ego vehicle during ground point segmentation)
    bool success = true;
    std::string ego_name;
    success &= nh.getParam("/vehicles/ego", ego_name);
    success &= nh.getParam("/vehicles/" + ego_name + "/geometric/height_ref_to_ground", height_ref_to_ground_);
    success &= nh.getParam("/vehicles/" + ego_name + "/geometric/length_ref_to_front_end", length_ref_to_front_end_);
    success &= nh.getParam("/vehicles/" + ego_name + "/geometric/length_ref_to_rear_end", length_ref_to_rear_end_);
    success &= nh.getParam("/vehicles/" + ego_name + "/geometric/width_ref_to_left_mirror", width_ref_to_left_mirror_);
    success &=
        nh.getParam("/vehicles/" + ego_name + "/geometric/width_ref_to_right_mirror", width_ref_to_right_mirror_);
    if (!success)
        throw std::runtime_error("Not all vehicle parameters are available");

    // init ROS subscribers and publishers
    pub_raw_firings = nh.advertise<sensor_msgs::PointCloud2>("raw_firings", 1000);
    pub_ground_point_segmentation = nh.advertise<sensor_msgs::PointCloud2>("streaming_ground_point_segmentation", 1000);
    pub_instance_segmentation = nh.advertise<sensor_msgs::PointCloud2>("streaming_instance_segmentation", 1000);
    pub_clusters = nh.advertise<sensor_msgs::PointCloud2>("streaming_clusters", 1000);

    // init TF synchronizer
    tf_synchronizer.setCallback(&StreamingClustering::onNewFiringsWithTfCallback, this);

    // init dynamic reconfigure
    reconfigure_server_.setCallback([this](StreamingClusteringConfig& config, uint32_t level)
                                    { callbackReconfigure(config, level); });
}

void StreamingClustering::reset()
{
    // reset
    last_update_ = ros::Time(0, 0);

    // shutdown workers
    insertion_thread_pool.shutdown();
    segmentation_thread_pool.shutdown();
    association_thread_pool.shutdown();
    tree_combination_thread_pool.shutdown();
    publishing_thread_pool.shutdown();

    // init/reset range image (implemented as ring buffer)
    range_image_.resize(ring_buffer_max_columns * num_rows);
    clearColumns(0, ring_buffer_max_columns - 1);
    ring_buffer_start_global_column_index = -1; // does not start at zero but at the minimum laser of first firing
    ring_buffer_end_global_column_index = -1;

    // reset members for streaming range image generation (srig)
    srig_previous_global_column_index_of_rearmost_laser = 0;
    srig_previous_global_column_index_of_foremost_laser = -1;
    srig_first_unfinished_global_column_index = -1;
    srig_reset_because_of_bad_start = false;

    // reset members for streaming ground point segmentation (sgps)
    sgps_previous_ground_points_.resize(num_rows, -1);
    sgps_next_ground_points_.resize(num_rows, std::make_shared<int64_t>(-1));

    // reset members for streaming clustering (sc)
    sc_first_unpublished_global_column_index = -1;
    sc_minimum_required_global_column_indices.clear();
    sc_unfinished_point_trees_.clear();
    sc_cluster_counter_ = 0;
    sc_inclination_angles_between_lasers_.resize(num_rows, std::nanf(""));

    // reset
    last_update_ = ros::Time(0, 0);
    first_firing_after_reset = true;

    // reset tf synchronizer
    tf_synchronizer.reset(odom_frame, sensor_frame);
    tf_synchronizer.waitForTransform(wait_for_tf);

    // re-initialize workers
    insertion_thread_pool.init([this](InsertionJob&& job)
                               { insertFiringIntoRangeImage(std::forward<InsertionJob>(job)); });
    segmentation_thread_pool.init([this](SegmentationJob&& job)
                                  { performGroundPointSegmentationForColumn(std::forward<SegmentationJob>(job)); });
    association_thread_pool.init([this](AssociationJob&& job)
                                 { associatePointsInColumn(std::forward<AssociationJob>(job)); });
    tree_combination_thread_pool.init([this](TreeCombinationJob&& job)
                                      { findFinishedTreesAndAssignSameId(std::forward<TreeCombinationJob>(job)); });
    publishing_thread_pool.init(
        [this](PublishingJob&& job) { collectPointsForCusterAndPublish(std::forward<PublishingJob>(job)); }, 3);

    // clear sensor input
    sensor_input_->reset();
}

void StreamingClustering::onNewFiringsCallback(const RawPoints::ConstPtr& firing)
{
    // all points in one firing have approximately the same stamp
    ros::Time stamp_current_firing;
    stamp_current_firing.fromNSec(firing->points[0].stamp);

    double input_latency = (ros::Time::now() - stamp_current_firing).toSec();
    if (input_latency > 0.1)
    {
        ROS_ERROR_STREAM("LATENCY FIRINGS: " << input_latency);

        ROS_ERROR_STREAM("RING BUFFER BOUNDS: " << ring_buffer_start_global_column_index << ", "
                                                << ring_buffer_end_global_column_index);

        ROS_ERROR_STREAM("JOB QUEUES (INSERT, ASSOC, TREE, PUB): "
                         << insertion_thread_pool.get_number_of_unprocessed_jobs() << ", "
                         << association_thread_pool.get_number_of_unprocessed_jobs() << ", "
                         << tree_combination_thread_pool.get_number_of_unprocessed_jobs() << ", "
                         << publishing_thread_pool.get_number_of_unprocessed_jobs());
    }

    // dynamically save number of rows
    num_rows = static_cast<int>(firing->points.size());

    // handle time jumps and bad start condition
    double dt = (stamp_current_firing - last_update_).toSec();
    if (srig_reset_because_of_bad_start || (!first_firing_after_reset && std::abs(dt) > 0.1))
    {
        ROS_WARN_STREAM("Detected jump in time ("
                        << dt << ") or bad start condition was encountered. Reset streaming clustering.");
        reset();
        return;
    }
    first_firing_after_reset = false;
    last_update_ = stamp_current_firing;

    // wait for transforms
    tf_synchronizer.addMessage(stamp_current_firing, firing);

    publishFiring(firing);

    /*std::cout << "JOB QUEUES (INSERT, SEGMENT, ASSOC, TREE, PUB): "
              << insertion_thread_pool.get_number_of_unprocessed_jobs() << ", "
              << segmentation_thread_pool.get_number_of_unprocessed_jobs() << ", "
              << association_thread_pool.get_number_of_unprocessed_jobs() << ", "
              << tree_combination_thread_pool.get_number_of_unprocessed_jobs() << ", "
              << publishing_thread_pool.get_number_of_unprocessed_jobs() << std::endl;*/
}

void StreamingClustering::onNewFiringsWithTfCallback(const RawPoints::ConstPtr& firing,
                                                     const geometry_msgs::TransformStamped& odom_from_sensor)
{
    insertion_thread_pool.enqueue({firing, odom_from_sensor});
}

void StreamingClustering::insertFiringIntoRangeImage(InsertionJob&& job)
{
    // all points in one firing have approximately the same stamp
    ros::Time stamp_current_firing;
    stamp_current_firing.fromNSec(job.firing->points[0].stamp);

    // get odom from sensor in order to transform points from sensor into odom coordinate system
    tf2::Transform odom_from_sensor;
    fromMsg(job.odom_from_sensor.transform, odom_from_sensor);
    fromMsg(job.odom_from_sensor.transform.translation, srig_sensor_position);

    // sensor position
    sgps_sensor_position.x = static_cast<float>(srig_sensor_position.x());
    sgps_sensor_position.y = static_cast<float>(srig_sensor_position.y());
    sgps_sensor_position.z = static_cast<float>(srig_sensor_position.z());

    // get base_link from odom in order to transform points from odom into vehicle
    // coordinates (important to find points on roof, etc.)
    try
    {
        geometry_msgs::TransformStamped tf = tf_synchronizer.getTfBuffer().lookupTransform(
            "base_link", odom_frame, job.odom_from_sensor.header.stamp); // use same stamp
        tf2::fromMsg(tf.transform, sgps_base_link_from_odom_);
    }
    catch (tf2::TransformException& ex)
    {
        // it can still happen that only one transform is in buffer so no interpolation is possible
        return;
    }

    // keep track of the global column indices of the foremost and rearmost laser (w.r.t. azimuth angle clockwise =
    // rotation direction of lidar sensor) in this firing
    int64_t global_column_index_of_foremost_laser = -1;
    int64_t global_column_index_of_rearmost_laser = -1;

    // rotation index
    int64_t previous_rotation_index_of_rearmost_laser =
        srig_previous_global_column_index_of_rearmost_laser / num_columns;

    // process firing from top to bottom
    for (int row_index = 0; row_index < job.firing->points.size(); row_index++)
    {
        // obtain point
        const RawPoint& raw_point = job.firing->points[row_index];
        tf2::Vector3 p(raw_point.x, raw_point.y, raw_point.z);

        if (std::isnan(p.x()))
            continue;

        // transform point into odom
        tf2::Vector3 p_odom = odom_from_sensor * p;

        // get point relative to sensor origin
        tf2::Vector3 p_odom_rel = p_odom - srig_sensor_position;

        // calculate azimuth angle w.r.t odom frame
        float azimuth_angle = std::atan2(static_cast<float>(p_odom_rel.y()), static_cast<float>(p_odom_rel.x()));

        // calculate azimuth angle which starts at negative X-axis with 0 and increases with ongoing lidar rotation
        // to 2 pi, which is more intuitive and important for fast array index calculation
        float increasing_azimuth_angle = -azimuth_angle + static_cast<float>(M_PI);

        // global column index
        int column_index_within_rotation = static_cast<int>(increasing_azimuth_angle / srig_azimuth_width_per_column);
        int64_t global_column_index =
            previous_rotation_index_of_rearmost_laser * num_columns + column_index_within_rotation;

        // check if we hit negative x-axis with w.r.t. previous firing
        int column_index_within_rotation_of_previous_rearmost_laser =
            static_cast<int>(srig_previous_global_column_index_of_rearmost_laser % num_columns);
        int column_diff = column_index_within_rotation - column_index_within_rotation_of_previous_rearmost_laser;
        int columns_of_half_rotation = num_columns / 2;
        int rotation_index_offset = 0;
        if (column_diff < -columns_of_half_rotation)
        {
            global_column_index += num_columns; // add one rotation
            rotation_index_offset = 1;
        }
        else if (srig_previous_global_column_index_of_rearmost_laser > 0 && column_diff > columns_of_half_rotation)
        {
            // In very rare cases this can happen because the minimum azimuth of rearmost laser can be smaller than
            // that of previous firing (most probably due to ego motion correction).
            // This gets only tricky when srig_previous_global_column_index_of_rearmost_laser % num_columns == 0 and
            // azimuth of rearmost laser in current firing is smaller than previous one.
            // So we have to subtract one rotation.
            global_column_index -= num_columns; // subtract one rotation
            rotation_index_offset = -1;
        }

        // local column index
        int local_column_index = static_cast<int>(global_column_index % ring_buffer_max_columns);

        // get correct point
        Point* point = &range_image_[local_column_index * num_rows + row_index]; // column major order

        // calculate continuous azimuth angle (even if we move it to the next cell, this value remains the same)
        double continuous_azimuth_angle =
            (2 * M_PI) * static_cast<double>(previous_rotation_index_of_rearmost_laser + rotation_index_offset) +
            increasing_azimuth_angle;

        // in case this cell is already occupied, try next column
        auto distance = static_cast<float>(p_odom_rel.length());
        if (!std::isnan(point->distance) && !std::isnan(distance))
        {
            int next_local_column_index = local_column_index + 1;
            if (next_local_column_index >= ring_buffer_max_columns)
                next_local_column_index -= ring_buffer_max_columns;
            Point* next_point = &range_image_[next_local_column_index * num_rows + row_index];
            if (std::isnan(next_point->distance))
            {
                point = next_point;
                local_column_index = next_local_column_index;
                global_column_index++;
            }
        }

        // avoid that a valid cell (non-nan) is overwritten by a nan or more distant value
        if (!std::isnan(point->distance) && (std::isnan(distance) || distance >= point->distance))
            continue;

        // do not insert into columns that are already cleared (or going to be cleared)
        int64_t ring_buffer_start_global_column_index_copy = ring_buffer_start_global_column_index;
        if (ring_buffer_start_global_column_index_copy >= 0 &&
            global_column_index < ring_buffer_start_global_column_index_copy)
        {
            ROS_WARN_STREAM(
                "Ignore point of firing because it would be inserted into a cleared column. Wanted to insert at "
                << global_column_index << ", but first ring buffer start is already at "
                << ring_buffer_start_global_column_index_copy << " (row index: " << row_index << ")");
            continue;
        }

        // fill point data
        point->xyz.x = static_cast<float>(p_odom.x());
        point->xyz.y = static_cast<float>(p_odom.y());
        point->xyz.z = static_cast<float>(p_odom.z());
        point->distance = distance;
        point->azimuth_angle = azimuth_angle;
        point->inclination_angle = std::asin(static_cast<float>(p_odom_rel.z()) / point->distance);
        point->continuous_azimuth_angle = continuous_azimuth_angle; // omitted cells will be filled again later
        point->global_column_index = global_column_index;           // omitted cells will be filled again later
        point->local_column_index = local_column_index;
        point->row_index = row_index;
        point->intensity = raw_point.intensity;
        point->stamp.fromNSec(raw_point.stamp);

        // keep track of global column index of rearmost & foremost laser
        if (global_column_index_of_rearmost_laser < 0 || global_column_index < global_column_index_of_rearmost_laser)
            global_column_index_of_rearmost_laser = global_column_index;
        if (global_column_index_of_foremost_laser < 0 || global_column_index > global_column_index_of_foremost_laser)
            global_column_index_of_foremost_laser = global_column_index;
    }

    // if for this firing a minimum/maximum column index was found then use it as the new one
    if (global_column_index_of_rearmost_laser >= 0 && global_column_index_of_foremost_laser >= 0)
    {
        // if the azimuth range of the firing covers more than 180 degrees, this means that the firing is
        // intersected with negative x-axis (this means that the range image was incorrectly filled -> reset)
        if ((global_column_index_of_foremost_laser - global_column_index_of_rearmost_laser) > num_columns / 2)
        {
            ROS_WARN_STREAM("Very first firing after reset intersects with negative x-axis: " +
                            std::to_string(global_column_index_of_rearmost_laser) + ", " +
                            std::to_string(global_column_index_of_foremost_laser) + ", " +
                            std::to_string(srig_previous_global_column_index_of_rearmost_laser) +
                            ". This is invalid. Reset streaming clustering on next message.");
            srig_reset_because_of_bad_start = true;
            return;
        }

        srig_previous_global_column_index_of_rearmost_laser = global_column_index_of_rearmost_laser;
        srig_previous_global_column_index_of_foremost_laser = global_column_index_of_foremost_laser;
    }

    // there is no information about minimum and maximum global column index
    if (srig_previous_global_column_index_of_foremost_laser < 0)
        return;

    // initialize start of ring buffer
    if (ring_buffer_start_global_column_index == -1)
    {
        ring_buffer_start_global_column_index = srig_previous_global_column_index_of_rearmost_laser;
        sc_first_unpublished_global_column_index = srig_previous_global_column_index_of_rearmost_laser;
    }

    // update end of ring buffer (maximum global column index ever seen)
    if (srig_previous_global_column_index_of_foremost_laser > ring_buffer_end_global_column_index)
        ring_buffer_end_global_column_index = srig_previous_global_column_index_of_foremost_laser;

    // start publishing at index of last laser of very first firing
    if (srig_first_unfinished_global_column_index == -1)
        srig_first_unfinished_global_column_index = srig_previous_global_column_index_of_rearmost_laser;

    // iterate over finished but unfinished columns and publish them
    while (srig_first_unfinished_global_column_index < srig_previous_global_column_index_of_rearmost_laser)
        segmentation_thread_pool.enqueue({srig_first_unfinished_global_column_index++});
}

void StreamingClustering::performGroundPointSegmentationForColumn(SegmentationJob&& job)
{
    int local_column_index = static_cast<int>(job.ring_buffer_current_global_column_index % ring_buffer_max_columns);

    // iterate rows from bottom to top and find ground points
    bool first_point_found = false;
    Point3D last_ground_point_wrt_sensor = Point3D(0, 0, height_sensor_to_ground_);
    Point3D previous_position, previous_position_wrt_sensor;
    int number_of_flat_points_after_obstacle = 0;
    int row_index_of_first_flat_point_after_obstacle = -1;
    bool obstacle_mode = false;
    float inclination_previous_laser = 0; // calculate difference between inclination angles for subsequent steps
    for (int row_index = num_rows - 1; row_index >= 0; row_index--)
    {
        // obtain point
        Point& point = range_image_[local_column_index * num_rows + row_index];

        // check if there is a problem with the ring buffer
        int64_t point_global_column_index_copy = point.global_column_index;
        if (point_global_column_index_copy != job.ring_buffer_current_global_column_index &&
            point_global_column_index_copy != -1)
            throw std::runtime_error(
                "This column is not cleared. Probably this means the ring buffer is full or there "
                "is some other issue with clearing (not cleared at all or written after clearing): " +
                std::to_string(point_global_column_index_copy) + ", " +
                std::to_string(job.ring_buffer_current_global_column_index) + ", " +
                std::to_string(ring_buffer_max_columns));

        // refill local/global column index because it was not filled for omitted cells
        point.global_column_index = job.ring_buffer_current_global_column_index;
        point.local_column_index =
            static_cast<int>(job.ring_buffer_current_global_column_index % ring_buffer_max_columns);

        // keep track of (differences between) the inclination angles of the lasers (for later processing steps)
        float inclination_current_laser = range_image_[local_column_index * num_rows + row_index].inclination_angle;
        float diff = inclination_current_laser - inclination_previous_laser;
        if (!std::isnan(diff))
            sc_inclination_angles_between_lasers_[row_index] = diff; // last value is useless but we do not use it
        inclination_previous_laser = inclination_current_laser;

        // skip NaN's
        if (std::isnan(point.distance))
        {
            // use inclination angle from previous column (it is useful to supplement nan cells with an inclination
            // angle in order to be able to break the while loop earlier during association
            if (sc_supplement_inclination_angle_for_nan_cells && row_index < num_rows - 1)
            {
                Point& point_below = range_image_[local_column_index * num_rows + (row_index + 1)];
                point.inclination_angle =
                    point_below.inclination_angle + sc_inclination_angles_between_lasers_[row_index];
            }
            // recalculate continuous azimuth for omitted/NaN cells (for later processing steps)
            point.continuous_azimuth_angle = (static_cast<double>(job.ring_buffer_current_global_column_index) + 0.5) *
                                             srig_azimuth_width_per_column;
            continue;
        }

        const Point3D& current_position = point.xyz;

        // special handling for points on ego vehicle surface
        tf2::Vector3 cur_point_in_base_link =
            sgps_base_link_from_odom_ * tf2::Vector3(current_position.x, current_position.y, current_position.z);
        if (cur_point_in_base_link.x() < length_ref_to_front_end_ &&
            cur_point_in_base_link.x() > length_ref_to_rear_end_ &&
            cur_point_in_base_link.y() < width_ref_to_left_mirror_ &&
            cur_point_in_base_link.y() > width_ref_to_right_mirror_)
        {
            point.ground_point_label = GP_EGO_VEHICLE;
            point.debug_ground_point_label = VIOLET;
            obstacle_mode = true;
            number_of_flat_points_after_obstacle = 0;
            row_index_of_first_flat_point_after_obstacle = -1;
            continue;
        }

        Point3D current_position_wrt_sensor = current_position - sgps_sensor_position;

        // special handling first point outside the ego bounding box
        if (!first_point_found)
        {
            // now we found the first point outside the ego vehicle box
            first_point_found = true;
            if (std::abs(cur_point_in_base_link.z() - height_ref_to_ground_) < config_.first_ring_max_height_diff)
            {
                point.ground_point_label = GP_GROUND;
                point.debug_ground_point_label = GREEN;
                point.height_over_ground = 0.f;
                last_ground_point_wrt_sensor = current_position_wrt_sensor;
                obstacle_mode = false;
            }
            else
            {
                point.ground_point_label = GP_OBSTACLE;
                point.debug_ground_point_label = RED;
                point.height_over_ground = current_position_wrt_sensor.z - last_ground_point_wrt_sensor.z;
                obstacle_mode = true;
            }
            previous_position = current_position;
            previous_position_wrt_sensor = current_position_wrt_sensor;
            continue;
        }

        float distance_xy = (current_position - previous_position).lengthXY();
        float slope = (current_position.z - previous_position.z) / distance_xy;
        if (std::abs(slope) < config_.max_slope &&
            previous_position_wrt_sensor.lengthXY() < current_position_wrt_sensor.lengthXY())
        {
            // quite flat
            if (obstacle_mode)
            {
                if (distance_xy < 2)
                {
                    // if there were enough flat points after obstacle we leave obstacle mode
                    number_of_flat_points_after_obstacle++;
                    if (row_index_of_first_flat_point_after_obstacle < 0)
                        row_index_of_first_flat_point_after_obstacle = row_index;

                    // obtain point
                    Point3D& first_position_after_obstacle =
                        range_image_[local_column_index * num_rows + row_index_of_first_flat_point_after_obstacle].xyz;

                    if (number_of_flat_points_after_obstacle >= config_.flat_section_after_obstacle_min_points &&
                        (current_position - first_position_after_obstacle).lengthXY() >=
                            config_.flat_section_after_obstacle_min_length)
                    {
                        // disable obstacle mode
                        obstacle_mode = false;
                        // and undo previous obstacle classifications
                        int previous_row_index = row_index + 1;
                        while (previous_row_index <= row_index_of_first_flat_point_after_obstacle)
                        {
                            int previous_data_index = local_column_index * num_rows + previous_row_index;
                            Point* previous_point = &range_image_[previous_data_index];
                            if (!std::isnan(previous_point->distance))
                            {
                                previous_point->ground_point_label = GP_GROUND;
                                previous_point->debug_ground_point_label = GREENYELLOW;
                                previous_point->height_over_ground = 0.f;
                            }
                            previous_row_index++;
                        }
                    }
                }
                else
                {
                    // large gap -> reset flat point count after obstacle
                    number_of_flat_points_after_obstacle = 0;
                    row_index_of_first_flat_point_after_obstacle = -1;
                }
            }
        }
        else
        {
            point.ground_point_label = GP_OBSTACLE;
            // too steep
            obstacle_mode = true;
            // reset flat point count after obstacle
            number_of_flat_points_after_obstacle = 0;
            row_index_of_first_flat_point_after_obstacle = -1;
        }

        if (obstacle_mode)
        {
            point.ground_point_label = GP_OBSTACLE;
            point.debug_ground_point_label = RED;
            if (current_position_wrt_sensor.lengthXY() - last_ground_point_wrt_sensor.lengthXY() > -10)
                point.height_over_ground = current_position_wrt_sensor.z - last_ground_point_wrt_sensor.z;
        }
        else
        {
            point.ground_point_label = GP_GROUND;
            point.debug_ground_point_label = GREEN;
            point.height_over_ground = 0.f;
            last_ground_point_wrt_sensor = current_position_wrt_sensor;
        }

        previous_position = current_position;
        previous_position_wrt_sensor = current_position_wrt_sensor;
    }

    // iterate again from bottom to top and find remaining obstacle points
    for (int row_index = num_rows - 1; row_index >= 0; row_index--)
    {
        int current_data_index_ri = local_column_index * num_rows + row_index; // column major
        Point& point = range_image_[current_data_index_ri];

        if (std::isnan(point.distance) || point.ground_point_label == GP_GROUND)
            continue;

        // get left ground point
        int64_t global_column_index_of_left_ground_point = sgps_previous_ground_points_[row_index];
        int64_t col_diff_left = job.ring_buffer_current_global_column_index - global_column_index_of_left_ground_point;
        if (global_column_index_of_left_ground_point < 0 || col_diff_left >= num_columns / 4)
            continue;

        point.local_column_index_of_left_ground_neighbor =
            static_cast<int>(global_column_index_of_left_ground_point % ring_buffer_max_columns);
        Point left_neighbor_point =
            range_image_[point.local_column_index_of_left_ground_neighbor * num_rows + row_index];

        // get right ground point

        // get same cell of previous rotation since is has maybe saved its next ground neighbor
        int64_t global_column_index_previous_rotation = job.ring_buffer_current_global_column_index - num_columns;
        if (global_column_index_previous_rotation < 0)
            continue;
        int local_column_index_previous_rotation =
            static_cast<int>(global_column_index_previous_rotation % ring_buffer_max_columns);
        Point& same_point_of_previous_rotation =
            range_image_[local_column_index_previous_rotation * num_rows + row_index];

        // obtain next ground neighbor
        if (!same_point_of_previous_rotation.pointer_to_global_column_index_of_next_ground_point)
            continue;
        int64_t global_column_index_of_right_ground_point =
            *(same_point_of_previous_rotation.pointer_to_global_column_index_of_next_ground_point);
        int64_t col_diff_right =
            job.ring_buffer_current_global_column_index - global_column_index_of_right_ground_point;
        if (global_column_index_of_right_ground_point < 0 || col_diff_right <= static_cast<int>(0.75 * num_columns) ||
            col_diff_right >= num_columns)
            continue;
        point.local_column_index_of_right_ground_neighbor =
            static_cast<int>(global_column_index_of_right_ground_point % ring_buffer_max_columns);
        Point right_neighbor_point =
            range_image_[point.local_column_index_of_right_ground_neighbor * num_rows + row_index];

        // create a line between those neighbors
        const Point3D& point_position = point.xyz;
        const Point3D& neighbor_left_position = left_neighbor_point.xyz;
        const Point3D& neighbor_right_position = right_neighbor_point.xyz;
        Point3D left_to_right = neighbor_right_position - neighbor_left_position;
        Point3D left_to_point = point_position - neighbor_left_position;

        // get point of line which is closest to current point
        float lambda = (left_to_point * left_to_right) / left_to_right.lengthSquared();
        Point3D closest_position_on_line = (neighbor_left_position + (lambda * left_to_right));

        // new check if they are close enough
        Point3D point_to_closest_point_on_line = closest_position_on_line - point_position;
        if (point_to_closest_point_on_line.xy().length() < config_.max_xy_distance_to_connection_line &&
            std::abs(point_to_closest_point_on_line.z) < config_.max_z_distance_to_connection_line)
        {
            point.debug_ground_point_label = YELLOW;
            point.ground_point_label = GP_GROUND;
            point.height_over_ground = 0.f;
        }
    }

    for (int row_index = num_rows - 1; row_index >= 0; row_index--)
    {
        int current_data_index_ri = local_column_index * num_rows + row_index; // column major
        Point& point = range_image_[current_data_index_ri];

        // save the latest ground point for each row (TODO: points with ground label shouldn't be nan)
        if (!std::isnan(point.xyz.z) &&
            (point.debug_ground_point_label == GREENYELLOW || point.debug_ground_point_label == GREEN))
        {
            sgps_previous_ground_points_[row_index] = job.ring_buffer_current_global_column_index;
            *(sgps_next_ground_points_[row_index]) = job.ring_buffer_current_global_column_index;
            sgps_next_ground_points_[row_index] = std::make_shared<int64_t>(-1);
        }

        // save for each point a pointer to the next ground point (will be determined in the future!)
        point.pointer_to_global_column_index_of_next_ground_point = sgps_next_ground_points_[row_index];

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
        if (point.distance < 1.5 * config_.max_distance)
        {
            point.is_ignored = true;
            continue;
        }

        // ignore this point if the distance in combination with inclination diff can't be below distance threshold
        if (config_.ignore_points_with_too_big_inclination_angle_diff && row_index < (num_rows - 1) &&
            std::atan2(config_.max_distance, point.distance) < sc_inclination_angles_between_lasers_[row_index])
        {
            point.is_ignored = true;
            continue;
        }

        // ignore this points in a chessboard pattern
        if (config_.ignore_points_in_chessboard_pattern)
        {
            bool column_even = point.global_column_index % 2 == 0;
            bool row_even = row_index % 2 == 0;
            if ((column_even && !row_even) || (!column_even && row_even))
            {
                point.is_ignored = true;
                continue;
            }
        }
    }

    publishColumns(job.ring_buffer_current_global_column_index,
                   job.ring_buffer_current_global_column_index,
                   pub_ground_point_segmentation);

    // lets enqueue the association job for this column to do it in a separate thread
    association_thread_pool.enqueue({job.ring_buffer_current_global_column_index});
}

void StreamingClustering::associatePointsInColumn(AssociationJob&& job)
{
    // collect new point trees
    std::list<RangeImageIndex> new_unfinished_point_trees{};

    // keep track of the current minimum azimuth angle of the current column
    double current_minimum_continuous_azimuth_angle = std::numeric_limits<double>::max();

    // get local start index of ring buffer start TODO: Ensure that this column is not cleared during this run
    int ring_buffer_first_local_column_index =
        static_cast<int>(sc_first_unpublished_global_column_index % ring_buffer_max_columns);

    int ring_buffer_current_local_column_index =
        static_cast<int>(job.ring_buffer_current_global_column_index % ring_buffer_max_columns);

    for (int row_index = 0; row_index < num_rows; row_index++)
    {
        // get current point
        Point& point = range_image_[ring_buffer_current_local_column_index * num_rows + row_index];

        // keep track of the current minimum continuous azimuth angle
        if (point.continuous_azimuth_angle < current_minimum_continuous_azimuth_angle)
            current_minimum_continuous_azimuth_angle = point.continuous_azimuth_angle;

        // check whether point should be ignored
        if (point.is_ignored)
            continue;

        // create index of this point
        RangeImageIndex index{static_cast<uint16_t>(row_index), ring_buffer_current_local_column_index};

        // calculate minimum possible azimuth angle
        float max_angle_diff = std::asin(max_distance_ / point.distance);
        double min_possible_continuous_azimuth_angle = point.continuous_azimuth_angle - max_angle_diff;

        // go left each column until azimuth angle difference gets too large
        int num_steps_back = 0;
        int64_t other_column_index = ring_buffer_current_local_column_index;
        while (num_steps_back <= config_.max_steps_in_row)
        {
            bool at_least_one_point_of_this_column_in_azimuth_range = false;
            for (int direction = -1; direction <= 1; direction += 2)
            {
                // do not go down in first column (these points are not associated to tree yet!)
                if (direction == 1 && num_steps_back == 0)
                    continue;

                // go up/down each row until the inclination angle difference gets too large
                int num_steps_vertical = direction == 1 || num_steps_back == 0 ? 1 : 0;
                int other_row_index = direction == 1 || num_steps_back == 0 ? row_index + direction : row_index;
                while (other_row_index >= 0 && other_row_index < num_rows &&
                       num_steps_vertical <= config_.max_steps_in_column)
                {
                    // get other point
                    Point& point_other = range_image_[other_column_index * num_rows + other_row_index];

                    // count number of visited points for analyzing
                    point.number_of_visited_neighbors += 1;

                    // no cluster can be associated because the inclination angle diff gets too large
                    if (std::abs(point_other.inclination_angle - point.inclination_angle) > max_angle_diff)
                        break;

                    // if this point is still in azimuth range then also search one column before
                    if (point_other.continuous_azimuth_angle >= min_possible_continuous_azimuth_angle)
                        at_least_one_point_of_this_column_in_azimuth_range = true;
                    else
                    {
                        other_row_index += direction;
                        num_steps_vertical++;
                        continue;
                    }

                    // if this point is ignored or has already the same tree root then do nothing (*1)
                    if (point_other.is_ignored ||
                        (point.tree_root_.column_index >= 0 && point_other.tree_root_ == point.tree_root_))
                    {
                        other_row_index += direction;
                        num_steps_vertical++;
                        continue;
                    }

                    // calculate the distance to other point
                    float distance_squared = (point.xyz - point_other.xyz).lengthSquared();

                    // if distance is small enough then try to associate to tree
                    if (distance_squared < max_distance_squared_)
                    {
                        if (point.tree_root_.column_index == -1)
                        {
                            // this point is not associated to any tree -> associate it
                            point.tree_root_ = point_other.tree_root_;
                            point_other.child_points.push_back(index);

                            // new point was associated to tree -> check if finish time will be delayed
                            Point& point_root = range_image_[point_other.tree_root_.column_index * num_rows +
                                                             point_other.tree_root_.row_index];
                            point_root.finished_at_continuous_azimuth_angle =
                                std::max(point_root.finished_at_continuous_azimuth_angle,
                                         point.continuous_azimuth_angle + max_angle_diff);
                            point_root.tree_num_points++;
                        }
                        else
                        {
                            // tree root of this point must be different to the other point (because of check above
                            // *1) so we must add a mutual link between the trees roots

                            // get tree root of current and other point
                            Point& point_root =
                                range_image_[point.tree_root_.column_index * num_rows + point.tree_root_.row_index];
                            Point& point_root_other = range_image_[point_other.tree_root_.column_index * num_rows +
                                                                   point_other.tree_root_.row_index];

                            // add mutual link
                            point_root.associated_trees.insert(point_other.tree_root_);
                            point_root_other.associated_trees.insert(point.tree_root_);
                        }
                    }

                    // stop searching if point was already associated and minimum number of columns were processed
                    if (point.tree_root_.column_index != -1 && config_.stop_after_association_enabled &&
                        num_steps_vertical >= config_.stop_after_association_min_steps)
                        break;

                    other_row_index += direction;
                    num_steps_vertical++;
                }
            }

            // stop searching if point was already associated and minimum number of points were processed
            if (point.tree_root_.column_index != -1 && config_.stop_after_association_enabled &&
                num_steps_back >= config_.stop_after_association_min_steps)
                break;

            // stop searching in previous columns because in previous run all points were already out of angle range
            if (num_steps_back > 0 && !at_least_one_point_of_this_column_in_azimuth_range)
                break;

            // stop searching if we are at the beginning of the ring buffer
            if (other_column_index == ring_buffer_first_local_column_index)
                break;

            other_column_index--;
            num_steps_back++;

            // jump to the end of the ring buffer
            if (other_column_index < 0)
                other_column_index += ring_buffer_max_columns;
        }

        if (point.tree_root_.column_index == -1)
        {
            // this point could not be associated to any tree -> init new tree
            point.tree_root_ = index;

            // calculate finish time (will be increased when new points are associated to tree)
            point.finished_at_continuous_azimuth_angle = point.continuous_azimuth_angle + max_angle_diff;

            // new tree has 1 point
            point.tree_num_points = 1;

            // create shortcut to this tree root
            new_unfinished_point_trees.push_back(index);
        }
    }

    // pass this job to the next thread
    TreeCombinationJob next_job;
    next_job.ring_buffer_current_global_column_index = job.ring_buffer_current_global_column_index;
    next_job.new_unfinished_point_trees = std::move(new_unfinished_point_trees);
    next_job.current_minimum_continuous_azimuth_angle = current_minimum_continuous_azimuth_angle;
    tree_combination_thread_pool.enqueue(std::move(next_job));
}

void StreamingClustering::findFinishedTreesAndAssignSameId(TreeCombinationJob&& job)
{
    sc_unfinished_point_trees_.splice(sc_unfinished_point_trees_.end(), std::move(job.new_unfinished_point_trees));

    std::list<std::list<RangeImageIndex>> trees_per_finished_cluster;
    std::list<int64_t> finished_cluster_ids;

    std::list<RangeImageIndex> trees_collected_for_current_cluster;
    std::list<RangeImageIndex> trees_to_visit_for_current_cluster;
    for (RangeImageIndex& tree_root_index : sc_unfinished_point_trees_)
    {
        Point& point_root = range_image_[tree_root_index.column_index * num_rows + tree_root_index.row_index];
        if (point_root.visited_at_continuous_azimuth_angle == job.current_minimum_continuous_azimuth_angle)
            continue; // this tree was already visited
        trees_collected_for_current_cluster.clear();
        trees_to_visit_for_current_cluster.clear();
        trees_to_visit_for_current_cluster.emplace_back(tree_root_index);
        uint32_t cluster_num_points = 0;
        bool cluster_has_at_least_one_unfinished_tree = false;
        while (!trees_to_visit_for_current_cluster.empty())
        {
            RangeImageIndex cur_tree_root_index = trees_to_visit_for_current_cluster.front();
            trees_to_visit_for_current_cluster.pop_front();
            Point& cur_point_root =
                range_image_[cur_tree_root_index.column_index * num_rows + cur_tree_root_index.row_index];
            if (cur_point_root.finished_at_continuous_azimuth_angle > job.current_minimum_continuous_azimuth_angle)
                cluster_has_at_least_one_unfinished_tree = true;
            if (cur_point_root.visited_at_continuous_azimuth_angle == job.current_minimum_continuous_azimuth_angle)
                continue; // cluster was already visited
            // cur_point_root.id = cluster_counter_;
            cur_point_root.visited_at_continuous_azimuth_angle = job.current_minimum_continuous_azimuth_angle;
            trees_collected_for_current_cluster.push_back(cur_tree_root_index);
            cluster_num_points += cur_point_root.tree_num_points;

            // iterate over the edges of the current vertex
            for (const RangeImageIndex& other_tree_root_index : cur_point_root.associated_trees)
            {
                Point& other_point_root =
                    range_image_[other_tree_root_index.column_index * num_rows + other_tree_root_index.row_index];
                if (other_point_root.visited_at_continuous_azimuth_angle !=
                    job.current_minimum_continuous_azimuth_angle)
                    trees_to_visit_for_current_cluster.push_back(other_tree_root_index);
            }
        }

        if (trees_collected_for_current_cluster.empty() || cluster_has_at_least_one_unfinished_tree)
            continue;

        // we found a completely finished cluster; now we mark its trees finished and prepare them for publishing
        for (const RangeImageIndex& cur_tree_root_index : trees_collected_for_current_cluster)
        {
            // mark current tree root as finished
            Point& cur_point_root =
                range_image_[cur_tree_root_index.column_index * num_rows + cur_tree_root_index.row_index];
            cur_point_root.belongs_to_finished_cluster = true;
        }

        if (cluster_num_points > 20)
        {
            trees_per_finished_cluster.push_back(std::move(trees_collected_for_current_cluster));
            finished_cluster_ids.push_back(sc_cluster_counter_++);
        }
    }

    // erase finished trees belonging to finished clusters + keep track of the minimum global column index of tree roots
    int64_t minimum_required_global_column_index = std::numeric_limits<int64_t>::max();
    auto it_erase = sc_unfinished_point_trees_.begin();
    while (it_erase != sc_unfinished_point_trees_.end())
    {
        Point& cur_point_root = range_image_[it_erase->column_index * num_rows + it_erase->row_index];
        if (cur_point_root.global_column_index < minimum_required_global_column_index)
            minimum_required_global_column_index = cur_point_root.global_column_index;
        if (cur_point_root.belongs_to_finished_cluster)
            it_erase = sc_unfinished_point_trees_.erase(it_erase);
        else
            it_erase++;
    }

    // if there are no unfinished clusters then set the start index one after current global column index
    if (minimum_required_global_column_index == std::numeric_limits<int64_t>::max())
        minimum_required_global_column_index = job.ring_buffer_current_global_column_index + 1;

    // save the minimum required global column index of this job in order not to publish/clear the data too early
    {
        std::lock_guard<std::mutex> lock(sc_first_unfinished_global_column_index_mutex);
        sc_minimum_required_global_column_indices.push_back(minimum_required_global_column_index);
    }

    // create job for new clusters
    PublishingJob next_job;
    next_job.ring_buffer_current_global_column_index = job.ring_buffer_current_global_column_index;
    next_job.ring_buffer_min_required_global_column_index = minimum_required_global_column_index;
    next_job.cluster_ids = std::move(finished_cluster_ids);
    next_job.trees_per_finished_cluster = std::move(trees_per_finished_cluster);
    publishing_thread_pool.enqueue(std::move(next_job));
}

void StreamingClustering::collectPointsForCusterAndPublish(PublishingJob&& job)
{
    // keep track of minimum stamp for this message
    ros::Time min_stamp_for_this_msg(std::numeric_limits<int32_t>::max(), 0);

    // create buffer
    std::vector<Point> cluster_points;

    auto it_trees_per_finished_cluster = job.trees_per_finished_cluster.begin();
    for (int64_t cluster_id : job.cluster_ids)
    {
        // clear buffer
        cluster_points.clear();

        // collect minimum and maximum stamp for this cluster
        ros::Time min_stamp_for_this_cluster(std::numeric_limits<int32_t>::max(), 0);
        ros::Time max_stamp_for_this_cluster(0, 0);

        // collect all of its child points
        std::list<RangeImageIndex> points_to_visit_for_this_tree;
        for (const RangeImageIndex& cur_tree_root_index : *it_trees_per_finished_cluster)
        {
            points_to_visit_for_this_tree.clear();
            points_to_visit_for_this_tree.push_back(cur_tree_root_index);
            while (!points_to_visit_for_this_tree.empty())
            {
                RangeImageIndex cur_point_index = points_to_visit_for_this_tree.front();
                points_to_visit_for_this_tree.pop_front();
                Point& cur_point = range_image_[cur_point_index.column_index * num_rows + cur_point_index.row_index];
                cur_point.id = cluster_id;
                cluster_points.push_back(cur_point);
                if (cur_point.stamp < min_stamp_for_this_cluster)
                    min_stamp_for_this_cluster = cur_point.stamp;
                if (cur_point.stamp > max_stamp_for_this_cluster)
                    max_stamp_for_this_cluster = cur_point.stamp;

                // iterate over the edges of the current vertex
                for (const RangeImageIndex& child_point_index : cur_point.child_points)
                    points_to_visit_for_this_tree.push_back(child_point_index);
            }
        }

        // keep track of minimum stamp for this message
        if (min_stamp_for_this_cluster < min_stamp_for_this_msg)
            min_stamp_for_this_msg = min_stamp_for_this_cluster;

        // publish points
        if (cluster_points.size() > 20)
        {
            sensor_msgs::PointCloud2Ptr pc(new sensor_msgs::PointCloud2);
            createPointCloud2ForCluster(*pc, cluster_points, min_stamp_for_this_cluster, max_stamp_for_this_cluster);
            pub_clusters.publish(pc);
        }

        it_trees_per_finished_cluster++;
    }

    // update start of column index (this is challenging because this job could be finished earlier than a previous one
    // due to parallelization).
    int64_t first_unpublished_global_column_index_old;
    int64_t first_unpublished_global_column_index_new;
    int64_t ring_buffer_start_global_column_index_old;
    int64_t ring_buffer_start_global_column_index_new;
    {
        std::lock_guard<std::mutex> lock(sc_first_unfinished_global_column_index_mutex);

        // remove the minimum required global column index from the list, as this job (batch of clusters) is finished
        auto pos = std::lower_bound(sc_minimum_required_global_column_indices.begin(),
                                    sc_minimum_required_global_column_indices.end(),
                                    job.ring_buffer_min_required_global_column_index);
        if (pos != sc_minimum_required_global_column_indices.end() &&
            *pos == job.ring_buffer_min_required_global_column_index)
            sc_minimum_required_global_column_indices.erase(pos);
        else
            throw std::runtime_error("The minimum unprocessed column index is not available! This is a bug!");

        // start of the publishing/clear range
        ring_buffer_start_global_column_index_old = ring_buffer_start_global_column_index;
        first_unpublished_global_column_index_old = sc_first_unpublished_global_column_index;

        // find end of publish range
        if (!sc_minimum_required_global_column_indices.empty())
        {
            // set start of ring buffer to the minimum required global column index along all unpublished clusters
            sc_first_unpublished_global_column_index = sc_minimum_required_global_column_indices.front();
        }
        else
        {
            // no other unfinished clusters -> set ring buffer start to the minimum unprocessed column associated to
            // current batch of clusters
            sc_first_unpublished_global_column_index = job.ring_buffer_min_required_global_column_index;
        }

        // ensure that first unpublished global column index is never decreasing
        if (sc_first_unpublished_global_column_index < first_unpublished_global_column_index_old)
            throw std::runtime_error("This shouldn't happen, ring buffer is not allowed to increase at the front: " +
                                     std::to_string(sc_first_unpublished_global_column_index) + ", " +
                                     std::to_string(first_unpublished_global_column_index_old));

        // end of clear range: keep one rotation for visualization and for ground point segmentation ("next ground
        // point")
        ring_buffer_start_global_column_index = std::max(0l, sc_first_unpublished_global_column_index - num_columns);

        // save them
        ring_buffer_start_global_column_index_new = ring_buffer_start_global_column_index;
        first_unpublished_global_column_index_new = sc_first_unpublished_global_column_index;

        // published needs to be here in order to ensure that the columns are in sequence and not interrupted by another
        // publish call
        publishColumns(first_unpublished_global_column_index_old,
                       first_unpublished_global_column_index_new - 1,
                       pub_instance_segmentation);
    }
    clearColumns(ring_buffer_start_global_column_index_old, ring_buffer_start_global_column_index_new - 1);
}

void StreamingClustering::clearColumns(int64_t from_global_column_index, int64_t to_global_column_index)
{
    if (to_global_column_index < from_global_column_index)
        return;

    for (int64_t global_column_index = from_global_column_index; global_column_index <= to_global_column_index;
         global_column_index++)
    {
        int local_column_index = static_cast<int>(global_column_index % ring_buffer_max_columns);

        for (int row_index = 0; row_index < num_rows; row_index++)
        {
            // get correct point
            Point& point = range_image_[local_column_index * num_rows + row_index];

            // clear range image generation / general
            point.xyz = {std::nanf(""), std::nanf(""), std::nanf("")};
            point.distance = std::nanf("");
            point.azimuth_angle = std::nanf("");
            point.inclination_angle = std::nanf("");
            point.continuous_azimuth_angle = std::nan("");
            point.global_column_index = -1;
            point.local_column_index = -1;
            point.row_index = -1;
            point.intensity = 0;
            point.stamp = {0, 0};

            // clear ground point segmentation
            point.ground_point_label = 0;
            point.height_over_ground = std::nanf("");
            point.debug_ground_point_label = WHITE;
            // can be cleared because we ensure that at least one rotation is still available when
            // calculating the column range to clear
            point.pointer_to_global_column_index_of_next_ground_point.reset();
            point.local_column_index_of_left_ground_neighbor = 0;
            point.local_column_index_of_right_ground_neighbor = 0;

            // clear clustering
            point.is_ignored = false;
            point.finished_at_continuous_azimuth_angle = 0.f;
            point.child_points.clear();
            point.associated_trees.clear();
            point.tree_root_ = {0, -1};
            point.tree_num_points = 0;
            point.id = 0;
            point.visited_at_continuous_azimuth_angle = -1.;
            point.belongs_to_finished_cluster = false;
            point.number_of_visited_neighbors = 0;
        }
    }
}

void StreamingClustering::createPointCloud2ForCluster(sensor_msgs::PointCloud2& msg,
                                                      const std::vector<Point>& cluster_points,
                                                      const ros::Time& min_stamp_for_this_cluster,
                                                      const ros::Time& max_stamp_for_this_cluster)
{
    msg.header.stamp = sc_use_last_point_for_cluster_stamp ?
                           max_stamp_for_this_cluster :
                           min_stamp_for_this_cluster + (max_stamp_for_this_cluster - min_stamp_for_this_cluster) * 0.5;
    msg.header.frame_id = odom_frame;
    msg.width = cluster_points.size();
    msg.height = 1;

    PointCloud2Iterators container = prepareMessageAndCreateIterators(msg);

    int data_index_message = 0;
    for (const Point& point : cluster_points)
    {
        addPointToMessage(container, data_index_message, point);
        data_index_message++;
    }
}

void StreamingClustering::publishColumns(int64_t from_global_column_index,
                                         int64_t to_global_column_index,
                                         ros::Publisher& pub)
{
    if (to_global_column_index < from_global_column_index)
        return;

    if (pub.getNumSubscribers() == 0)
        return;

    int num_columns_to_publish = static_cast<int>(to_global_column_index - from_global_column_index) + 1;
    if (num_columns_to_publish <= 0)
        return;

    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header.frame_id = odom_frame;
    msg->width = num_columns_to_publish;
    msg->height = num_rows;

    PointCloud2Iterators container = prepareMessageAndCreateIterators(*msg);

    // keep track of minimum point stamp in this message
    ros::Time minimum_point_stamp{std::numeric_limits<int32_t>::max(), 0};

    for (int message_column_index = 0; message_column_index < msg->width; ++message_column_index)
    {
        int ring_buffer_local_column_index =
            static_cast<int>((from_global_column_index + message_column_index) % ring_buffer_max_columns);
        for (int row_index = 0; row_index < num_rows; ++row_index)
        {
            Point& point = range_image_[ring_buffer_local_column_index * num_rows + row_index];
            int data_index_message = row_index * static_cast<int>(msg->width) + message_column_index;
            addPointToMessage(container, data_index_message, point);

            if (!point.stamp.isZero() && point.stamp < minimum_point_stamp)
                minimum_point_stamp = point.stamp;
        }
    }

    msg->header.stamp = minimum_point_stamp;

    pub.publish(msg);
}

void StreamingClustering::publishFiring(const RawPoints::ConstPtr& firing)
{
    if (pub_raw_firings.getNumSubscribers() == 0)
        return;

    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header.frame_id = sensor_frame;
    msg->width = 1;
    msg->height = num_rows;

    PointCloud2Iterators container = prepareMessageAndCreateIterators(*msg);

    // keep track of minimum point stamp in this message
    uint64_t minimum_point_stamp = std::numeric_limits<uint64_t>::max();

    int data_index_message = 0;
    for (const RawPoint& point : firing->points)
    {
        addRawPointToMessage(container, data_index_message, point);

        if (point.stamp != 0 && point.stamp < minimum_point_stamp)
            minimum_point_stamp = point.stamp;

        data_index_message++;
    }

    if (minimum_point_stamp != std::numeric_limits<uint64_t>::max())
        msg->header.stamp = ros::Time().fromNSec(minimum_point_stamp);

    pub_raw_firings.publish(msg);
}

PointCloud2Iterators StreamingClustering::prepareMessageAndCreateIterators(sensor_msgs::PointCloud2& msg)
{
    msg.is_bigendian = false;
    msg.is_dense = false;

    sensor_msgs::PointCloud2Modifier output_modifier(msg);
    output_modifier.setPointCloud2Fields(25,
                                         "x",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "y",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "z",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "distance",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "azimuth_angle",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "inclination_angle",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "continuous_azimuth_angle",
                                         1,
                                         sensor_msgs::PointField::FLOAT64,
                                         "global_column_index",
                                         1,
                                         sensor_msgs::PointField::UINT32,
                                         "local_column_index",
                                         1,
                                         sensor_msgs::PointField::UINT16,
                                         "row_index",
                                         1,
                                         sensor_msgs::PointField::UINT16,
                                         "intensity",
                                         1,
                                         sensor_msgs::PointField::UINT8,
                                         "time_sec",
                                         1,
                                         sensor_msgs::PointField::UINT32,
                                         "time_nsec",
                                         1,
                                         sensor_msgs::PointField::UINT32,
                                         "ground_point_label",
                                         1,
                                         sensor_msgs::PointField::UINT8,
                                         "debug_ground_point_label",
                                         1,
                                         sensor_msgs::PointField::UINT8,
                                         "debug_local_column_index_of_left_ground_neighbor",
                                         1,
                                         sensor_msgs::PointField::INT32,
                                         "debug_local_column_index_of_right_ground_neighbor",
                                         1,
                                         sensor_msgs::PointField::INT32,
                                         "height_over_ground",
                                         1,
                                         sensor_msgs::PointField::FLOAT32,
                                         "finished_at_continuous_azimuth_angle",
                                         1,
                                         sensor_msgs::PointField::FLOAT64,
                                         "num_child_points",
                                         1,
                                         sensor_msgs::PointField::UINT16,
                                         "tree_root_row_index",
                                         1,
                                         sensor_msgs::PointField::UINT16,
                                         "tree_root_column_index",
                                         1,
                                         sensor_msgs::PointField::UINT8,
                                         "number_of_visited_neighbors",
                                         1,
                                         sensor_msgs::PointField::UINT32,
                                         "tree_id",
                                         1,
                                         sensor_msgs::PointField::UINT32,
                                         "id",
                                         1,
                                         sensor_msgs::PointField::UINT32);

    return {{msg, "x"},
            {msg, "y"},
            {msg, "z"},
            {msg, "distance"},
            {msg, "azimuth_angle"},
            {msg, "inclination_angle"},
            {msg, "continuous_azimuth_angle"},
            {msg, "global_column_index"},
            {msg, "local_column_index"},
            {msg, "row_index"},
            {msg, "intensity"},
            {msg, "time_sec"},
            {msg, "time_nsec"},
            {msg, "ground_point_label"},
            {msg, "debug_ground_point_label"},
            {msg, "debug_local_column_index_of_left_ground_neighbor"},
            {msg, "debug_local_column_index_of_right_ground_neighbor"},
            {msg, "height_over_ground"},
            {msg, "finished_at_continuous_azimuth_angle"},
            {msg, "num_child_points"},
            {msg, "tree_root_row_index"},
            {msg, "tree_root_column_index"},
            {msg, "number_of_visited_neighbors"},
            {msg, "tree_id"},
            {msg, "id"}};
}

void StreamingClustering::addPointToMessage(PointCloud2Iterators& container,
                                            int data_index_message,
                                            const Point& point) const
{
    *(container.iter_x_out + data_index_message) = point.xyz.x;
    *(container.iter_y_out + data_index_message) = point.xyz.y;
    *(container.iter_z_out + data_index_message) = point.xyz.z;
    *(container.iter_d_out + data_index_message) = point.distance;
    *(container.iter_a_out + data_index_message) = point.azimuth_angle;
    *(container.iter_ia_out + data_index_message) = point.inclination_angle;
    *(container.iter_ca_out + data_index_message) = point.continuous_azimuth_angle;
    *(container.iter_gc_out + data_index_message) = point.global_column_index;
    *(container.iter_lc_out + data_index_message) = point.local_column_index;
    *(container.iter_r_out + data_index_message) = point.row_index;
    *(container.iter_i_out + data_index_message) = point.intensity;
    *(container.iter_time_sec_out + data_index_message) = point.stamp.sec;
    *(container.iter_time_nsec_out + data_index_message) = point.stamp.nsec;
    *(container.iter_gp_label_out + data_index_message) = point.ground_point_label;
    *(container.iter_dbg_gp_label_out + data_index_message) = point.debug_ground_point_label;
    *(container.iter_dbg_c_n_left_out + data_index_message) = point.local_column_index_of_left_ground_neighbor;
    *(container.iter_dbg_c_n_right_out + data_index_message) = point.local_column_index_of_right_ground_neighbor;
    *(container.iter_height_over_ground_out + data_index_message) = point.height_over_ground;
    *(container.iter_finished_at_azimuth_angle + data_index_message) = point.finished_at_continuous_azimuth_angle;
    *(container.iter_num_child_points + data_index_message) = point.child_points.size();
    *(container.iter_tree_root_row_index + data_index_message) = point.tree_root_.row_index;
    *(container.iter_tree_root_column_index + data_index_message) = point.tree_root_.column_index;
    *(container.iter_number_of_visited_neighbors + data_index_message) = point.number_of_visited_neighbors;
    *(container.iter_tree_id + data_index_message) =
        static_cast<uint32_t>(point.tree_root_.column_index * num_rows + point.tree_root_.row_index);
    *(container.iter_id + data_index_message) = point.id;
}

void StreamingClustering::addRawPointToMessage(PointCloud2Iterators& container,
                                               int data_index_message,
                                               const RawPoint& point)
{
    *(container.iter_x_out + data_index_message) = point.x;
    *(container.iter_y_out + data_index_message) = point.y;
    *(container.iter_z_out + data_index_message) = point.z;
    *(container.iter_i_out + data_index_message) = point.intensity;

    ros::Time t;
    t.fromNSec(point.stamp);
    *(container.iter_time_sec_out + data_index_message) = t.sec;
    *(container.iter_time_nsec_out + data_index_message) = t.nsec;
}

void StreamingClustering::callbackReconfigure(StreamingClusteringConfig& config, uint32_t level)
{
    config_ = config;
    max_distance_ = static_cast<float>(config_.max_distance);
    max_distance_squared_ = static_cast<float>(config_.max_distance * config_.max_distance);
}

} // namespace streaming_clustering
