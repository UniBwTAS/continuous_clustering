#include <cmath>
#include <iomanip>
#include <iostream>

#include <continuous_clustering/evaluation/kitti_loader.hpp>

namespace continuous_clustering
{

KittiLoader::KittiLoader() = default;

std::vector<KittiPoint> KittiLoader::loadPointCloud(const Path& path)
{
    // load pointcloud
    std::vector<float> points_flattened = loadFlattenedPointCloud<float>(path);
    uint32_t num_points = points_flattened.size() / 4;
    std::vector<KittiPoint> points(num_points);
    int flattened_index = 0;
    for (int32_t i = 0; i < num_points; i++)
    {
        KittiPoint& p = points[i];
        p.x = points_flattened[flattened_index++];
        p.y = points_flattened[flattened_index++];
        p.z = points_flattened[flattened_index++];
        p.i = points_flattened[flattened_index++];
    }

    return points;
}

void KittiLoader::loadSemanticKittiLabels(const Path& path, std::vector<KittiPoint>& points)
{
    // load labels
    std::vector<uint16_t> labels_flattened = loadFlattenedPointCloud<uint16_t>(path);
    uint32_t num_points = labels_flattened.size() / 2;
    if (num_points != points.size())
        throw std::runtime_error("Number of points does not match (label/bin): " + std::to_string(num_points) + " / " +
                                 std::to_string(points.size()));
    int flattened_index = 0;
    for (int32_t i = 0; i < num_points; i++)
    {
        KittiPoint& p = points[i];
        p.semantic_label = labels_flattened[flattened_index++];
        p.instance_label = labels_flattened[flattened_index++];
    }
}

void KittiLoader::recoverLaserIndices(std::vector<KittiPoint>& points)
{
    // Points are stored in KITTI's .bin file in row-major order, however NaN values were omitted.
    // This means that no simple reshape is possible
    // Instead, we increment the row index whenever a jump in the azimuth values is detected
    // The points in a row are not ordered chronologically but somehow according to their atan2 angle (I don't know why)
    // from 0 -> pi -> -pi -> 0 (counterclockwise even though the velodyne rotates clockwise and starts at pi)
    int laser_index = 0;
    double prev_azimuth_monotonic = -1;
    int collected_points_for_this_row = 0;
    int max_columns = 0;
    for (auto& point : points)
    {
        // process unorganized point and get its azimuth angle
        double cur_azimuth = std::atan2(point.y, point.x);

        // the azimuth angles in a row from left to right are: 0 -> pi -> -pi -> 0
        // make them monotonically increasing from 0 -> 2 * pi
        double cur_azimuth_monotonic = cur_azimuth < 0 ? cur_azimuth + (2 * M_PI) : cur_azimuth;

        // detect if a new row is found whenever there is a jump in the azimuth values 2 * pi -> 0
        // especially the top rows are very sparse because they are over the horizon
        // therefore a jump of -0.7 rad (~ -40 degrees) backwards is enough (instead of -2 * pi)
        if (prev_azimuth_monotonic >= 0 and cur_azimuth_monotonic - prev_azimuth_monotonic < -0.7)
        {
            // increment row index
            laser_index++;
            if (laser_index >= RANGE_IMAGE_HEIGHT)
                break;

            // just some statistics
            max_columns = std::max(max_columns, collected_points_for_this_row);
            collected_points_for_this_row = 0;
        }

        // assign laser index
        point.laser_index = laser_index;

        // keep track of previous azimuth angle
        prev_azimuth_monotonic = cur_azimuth_monotonic;

        // just some statistics
        collected_points_for_this_row++;
    }

    if (laser_index + 1 != RANGE_IMAGE_HEIGHT)
        // throw std::runtime_error("Wrong number of rows found: " + std::to_string(laser_index + 1));
        std::cerr << "Wrong number of rows found: " << std::to_string(laser_index + 1) << std::endl;

    if (max_columns > RANGE_IMAGE_WIDTH)
        throw std::runtime_error("More points in a single row than expected: " + std::to_string(max_columns));
}

std::vector<KittiPoint> KittiLoader::generateRangeImage(const std::vector<KittiPoint>& unorganized_points,
                                                        bool shift_cell_if_already_occupied)
{
    // constants
    const double column_width = (2 * M_PI) / RANGE_IMAGE_WIDTH;

    // create range image and fill with NaNs
    std::vector<KittiPoint> organized_points(RANGE_IMAGE_HEIGHT * RANGE_IMAGE_WIDTH, KittiPoint());

    // count cell collisions for debugging
    // uint32_t num_collisions = 0;
    // uint32_t num_collisions_with_skip = 0;

    // Points for a single laser are not stored chronologically. Therefore, we try to sort them approximately
    // chronologically based on their azimuth angle and insert them into the range image
    for (int32_t original_index = 0; original_index < unorganized_points.size(); original_index++)
    {
        // process unorganized point and get its azimuth angle
        const KittiPoint& unorganized_point = unorganized_points[original_index];
        if (std::isnan(unorganized_point.x) || std::isnan(unorganized_point.y) || std::isnan(unorganized_point.z))
            continue;
        double cur_azimuth = std::atan2(unorganized_point.y, unorganized_point.x);

        // get column index
        auto column_index = static_cast<int>((M_PI - cur_azimuth) / column_width);

        // sometimes cur_azimuth is exactly -180 because point's x coordinate is exactly 0 -> column_index == num_cols
        if (column_index == RANGE_IMAGE_WIDTH)
            column_index--;

        // shift cell for current point if cell is already occupied
        if (shift_cell_if_already_occupied)
        {
            if (!std::isnan(organized_points[unorganized_point.laser_index * RANGE_IMAGE_WIDTH + column_index].x))
            {
                // cell already occupied
                // num_collisions++;
                int right_column_index = column_index + 1;
                if (right_column_index < RANGE_IMAGE_WIDTH &&
                    std::isnan(
                        organized_points[unorganized_point.laser_index * RANGE_IMAGE_WIDTH + right_column_index].x))
                {
                    // the cell on the right is not occupied
                    column_index = right_column_index;
                }
                else
                {
                    // the cell on the right is occupied
                    int left_column_index = column_index - 1;
                    if (left_column_index >= 0 &&
                        std::isnan(
                            organized_points[unorganized_point.laser_index * RANGE_IMAGE_WIDTH + left_column_index].x))
                    {
                        // the cell on the left is not occupied
                        column_index = left_column_index;
                    }
                    else
                    {
                        // the cell on the left is also occupied -> overwrite current cell
                        // num_collisions_with_skip++;
                    }
                }
            }
        }

        // insert point into range image
        uint32_t flattened_index = unorganized_point.laser_index * RANGE_IMAGE_WIDTH + column_index;
        KittiPoint& organized_point = organized_points[flattened_index];
        organized_point = unorganized_point;
        organized_point.original_kitti_index = original_index;
    }

    // std::cout << "Collisions: " << num_collisions << ", " << num_collisions_with_skip << ", "
    //           << unorganized_points.size() << std::endl;

    return organized_points;
}

void KittiLoader::undoEgoMotionCorrection(std::vector<KittiPoint>& corrected_points,
                                          uint64_t rotation_start_stamp,
                                          uint64_t rotation_end_stamp,
                                          const Eigen::Isometry3d& odom_from_velodyne_at_middle_of_rotation,
                                          const std::vector<StampedPose>& odom_from_velodyne)
{
    // constant
    uint64_t bin_resolution = 1000000; // 1ms

    // create lookup table for transforms with above resolution
    uint64_t duration = rotation_end_stamp - rotation_start_stamp;
    int num_bins = std::ceil(static_cast<double>(duration) / static_cast<double>(bin_resolution));
    std::vector<Eigen::Isometry3d> velodyne_from_velodyne(num_bins);
    for (int bin_index = 0; bin_index < num_bins; bin_index++)
    {
        // first transform point to odom from velodyne at the middle of the rotation and then transform it back to
        // velodyne frame at the point's stamp
        uint64_t stamp_at_bin = rotation_start_stamp + bin_index * bin_resolution + (bin_resolution / 2);
        velodyne_from_velodyne[bin_index] =
            interpolate(odom_from_velodyne, stamp_at_bin).pose.inverse() * odom_from_velodyne_at_middle_of_rotation;
    }

    // transform each point
    for (auto& p : corrected_points)
    {
        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
            continue;

        double fraction_of_scan_completed = (M_PI - std::atan2(p.y, p.x)) / (2.0 * M_PI);
        int bin_index = static_cast<int>((fraction_of_scan_completed * static_cast<double>(duration)) /
                                         static_cast<double>(bin_resolution));
        Eigen::Vector3d uncorrected_position = velodyne_from_velodyne[bin_index] * Eigen::Vector3d{p.x, p.y, p.z};
        p.x = static_cast<float>(uncorrected_position.x());
        p.y = static_cast<float>(uncorrected_position.y());
        p.z = static_cast<float>(uncorrected_position.z());
    }
}

Oxts KittiLoader::loadSingleOxfordMeasurement(const Path& path)
{
    std::ifstream is(path);
    std::string line;
    if (is.is_open())
    {
        std::getline(is, line);
        is.close();
    }
    else
        throw std::runtime_error("Unable to open: " + path.string());

    std::vector<std::string> v = split(line, ' ');

    return Oxts{0,
                std::stod(v[0]),
                std::stod(v[1]),
                std::stod(v[2]),
                std::stod(v[3]),
                std::stod(v[4]),
                std::stod(v[5]),
                std::stod(v[8]),
                std::stod(v[9]),
                std::stod(v[10])};
}

Eigen::Isometry3d KittiLoader::convertOxfordMeasurementToPose(const Oxts& oxts, double scale)
{
    // earth radius (approx.) in meters
    double const earth_radius{6378137.0};

    // mercator projection to get position
    double const tx{scale * earth_radius * M_PI * oxts.lon / 180.0};
    double const ty{scale * earth_radius * std::log(std::tan(M_PI * (90.0 + oxts.lat) / 360.0))};
    double const tz{oxts.alt};

    // use the Euler angles to get the rotation matrix
    Eigen::Matrix3d const R{Eigen::AngleAxisd(oxts.yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(oxts.pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(oxts.roll, Eigen::Vector3d::UnitX())};

    // combine the translation and rotation into a homogeneous transform
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    pose = R * pose;
    pose.translation() << tx, ty, tz;

    return pose;
}

std::vector<StampedPose> KittiLoader::getAllDynamicTransforms(const Path& path_oxford_folder,
                                                              int first_frame,
                                                              int last_frame,
                                                              const Eigen::Isometry3d& tf_oxford_from_x)
{
    // load oxford timestamps
    auto timestamps_oxford = KittiLoader::loadTimestampsRaw(path_oxford_folder / Path{"timestamps.txt"});

    // load all transforms odom from x (e.g. velodyne)
    double scale = 0.0;
    std::vector<StampedPose> transforms_odom_from_x;
    for (int frame_index = first_frame; frame_index <= last_frame; ++frame_index)
    {
        auto oxts = loadSingleOxfordMeasurement(path_oxford_folder / Path{"data"} /
                                                Path{padWithZeros(frame_index, 10) + ".txt"});
        if (scale == 0.0)
            scale = std::cos(oxts.lat * M_PI / 180.);
        auto tf_odom_from_oxford = convertOxfordMeasurementToPose(oxts, scale);
        auto tf_odom_from_velodyne = tf_odom_from_oxford * tf_oxford_from_x;
        transforms_odom_from_x.push_back({timestamps_oxford[frame_index], tf_odom_from_velodyne});
    }

    return transforms_odom_from_x;
}

std::vector<StampedPose> KittiLoader::makeTransformsRelativeToFirstTransform(const std::vector<StampedPose>& transforms)
{
    std::vector<StampedPose> relative_transforms;
    relative_transforms.reserve(transforms.size());
    auto first_tf = transforms.front();          // e.g. utm_from_odom
    auto first_tf_inv = first_tf.pose.inverse(); // e.g. odom_from_utm
    for (const auto& tf : transforms)
        relative_transforms.push_back({tf.stamp, first_tf_inv * tf.pose}); // e.g. odom_from_utm * utm_from_velodyne
    return relative_transforms;
}

StampedPose KittiLoader::interpolate(const std::vector<StampedPose>& transforms, uint64_t stamp)
{
    auto it_pose_after = std::lower_bound(transforms.begin(),
                                          transforms.end(),
                                          stamp,
                                          [](const StampedPose& pose, uint64_t stamp) { return pose.stamp < stamp; });
    if (it_pose_after == transforms.end())
        return {stamp, (it_pose_after - 1)->pose};
    else if (it_pose_after == transforms.begin())
        return {stamp, it_pose_after->pose};
    else
    {
        auto it_pose_before = it_pose_after - 1;
        double f = static_cast<double>(stamp - it_pose_before->stamp) /
                   static_cast<double>(it_pose_after->stamp - it_pose_before->stamp);

        // interpolate rotation
        Eigen::Quaterniond q_before{it_pose_before->pose.rotation()};
        Eigen::Quaterniond q_after{it_pose_after->pose.rotation()};
        Eigen::Quaterniond q = q_before.slerp(f, q_after);

        // interpolate translation
        Eigen::Vector3d t = (1 - f) * it_pose_before->pose.translation() + f * it_pose_after->pose.translation();

        // interpolated pose
        Eigen::Isometry3d interpolated_pose = Eigen::Isometry3d::Identity();
        interpolated_pose.linear() = q.toRotationMatrix();
        interpolated_pose.translation() = t;

        return {stamp, interpolated_pose};
    }
}

std::vector<StampedPose> KittiLoader::getAllDynamicTransforms(const Path& path_poses_file,
                                                              const std::vector<uint64_t>& timestamps,
                                                              const Eigen::Isometry3d& tf_cam0_from_x)
{
    std::ifstream is(path_poses_file);

    if (!is.is_open())
        throw std::runtime_error("Unable to open poses file: " + path_poses_file.string());

    Eigen::Isometry3d tf_odom_from_first_cam0{Eigen::Isometry3d::Identity()};
    tf_odom_from_first_cam0.linear() << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    std::vector<StampedPose> poses;
    std::string line;
    int i = 0;
    while (std::getline(is, line) && (timestamps.empty() || i < timestamps.size()))
    {
        std::vector<std::string> v = split(line, ' ');

        Eigen::Isometry3d tf_first_cam0_from_cam0{Eigen::Isometry3d::Identity()};
        tf_first_cam0_from_cam0.linear() << std::stod(v[0]), std::stod(v[1]), std::stod(v[2]), std::stod(v[4]),
            std::stod(v[5]), std::stod(v[6]), std::stod(v[8]), std::stod(v[9]), std::stod(v[10]);
        tf_first_cam0_from_cam0.translation() << std::stod(v[3]), std::stod(v[7]), std::stod(v[11]);

        Eigen::Isometry3d tf_odom_from_x = tf_odom_from_first_cam0 * tf_first_cam0_from_cam0 * tf_cam0_from_x;

        if (timestamps.empty())
            poses.push_back({0, tf_odom_from_x});
        else
            poses.push_back({timestamps[i], tf_odom_from_x});

        i++;
    }

    if (!timestamps.empty() && i != timestamps.size())
        throw std::runtime_error(
            "The number of poses (i.e. lines in poses.txt) does not match with number of timestamps.");

    return poses;
}

void KittiLoader::getStaticTransformAndProjectionMatrices(const Path& path_calib_file,
                                                          Eigen::Isometry3d& tf_cam0_from_velodyne,
                                                          Eigen::Affine3d& projection_matrix_cam0)
{
    std::ifstream calibration_is(path_calib_file);

    if (!calibration_is.is_open())
        throw std::runtime_error("Unable to open calibration file: " + path_calib_file.string());

    std::string line;

    // projection matrix P0
    std::getline(calibration_is, line);
    std::vector<std::string> v = split(line, ' ');
    projection_matrix_cam0.linear() << std::stod(v[1]), std::stod(v[2]), std::stod(v[3]), std::stod(v[5]),
        std::stod(v[6]), std::stod(v[7]), std::stod(v[9]), std::stod(v[10]), std::stod(v[11]);
    projection_matrix_cam0.translation() << std::stod(v[4]), std::stod(v[8]), std::stod(v[12]);

    /* // projection matrix P1
     std::getline(calibration_is, line);
     v = split(line, ' ');
     projection_matrix_cam1.linear() << std::stod(v[1]), std::stod(v[2]), std::stod(v[3]), std::stod(v[5]),
         std::stod(v[6]), std::stod(v[7]), std::stod(v[9]), std::stod(v[10]), std::stod(v[11]);
     projection_matrix_cam1.translation() << std::stod(v[4]), std::stod(v[8]), std::stod(v[12]);

     // projection matrix P2
     std::getline(calibration_is, line);
     v = split(line, ' ');
     projection_matrix_cam2.linear() << std::stod(v[1]), std::stod(v[2]), std::stod(v[3]), std::stod(v[5]),
         std::stod(v[6]), std::stod(v[7]), std::stod(v[9]), std::stod(v[10]), std::stod(v[11]);
     projection_matrix_cam2.translation() << std::stod(v[4]), std::stod(v[8]), std::stod(v[12]);

     // projection matrix P3
     std::getline(calibration_is, line);
     v = split(line, ' ');
     projection_matrix_cam3.linear() << std::stod(v[1]), std::stod(v[2]), std::stod(v[3]), std::stod(v[5]),
         std::stod(v[6]), std::stod(v[7]), std::stod(v[9]), std::stod(v[10]), std::stod(v[11]);
     projection_matrix_cam3.translation() << std::stod(v[4]), std::stod(v[8]), std::stod(v[12]);*/

    // transform matrix Tr
    std::getline(calibration_is, line);
    v = split(line, ' ');
    tf_cam0_from_velodyne.linear() << std::stod(v[1]), std::stod(v[2]), std::stod(v[3]), std::stod(v[5]),
        std::stod(v[6]), std::stod(v[7]), std::stod(v[9]), std::stod(v[10]), std::stod(v[11]);
    tf_cam0_from_velodyne.translation() << std::stod(v[4]), std::stod(v[8]), std::stod(v[12]);
}

Eigen::Isometry3d KittiLoader::loadStaticTransform(const Path& calibration_file)
{
    std::ifstream calibration_is(calibration_file);

    if (!calibration_is.is_open())
        throw std::runtime_error("Unable to open camera calibration file: " + calibration_file.string());

    std::string line;

    // skip first line which only contains meta information
    std::getline(calibration_is, line);

    // rotation matrix R
    Eigen::Matrix3d R;
    std::getline(calibration_is, line);
    std::vector<std::string> v = split(line, ' ');
    R << std::stod(v[1]), std::stod(v[2]), std::stod(v[3]), std::stod(v[4]), std::stod(v[5]), std::stod(v[6]),
        std::stod(v[7]), std::stod(v[8]), std::stod(v[9]);

    // translation vector T
    Eigen::Vector3d T;
    std::getline(calibration_is, line);
    v = split(line, ' ');
    T << std::stod(v[1]), std::stod(v[2]), std::stod(v[3]);

    // transform from lidar frame to camera_00
    Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
    transform = R * transform;
    transform.translation() = T;

    return transform;
}

Eigen::Isometry3d KittiLoader::loadStaticTransformVelodyneFromOxford(const Path& calibration_folder)
{
    return loadStaticTransform(Path{calibration_folder} / Path{"calib_imu_to_velo.txt"});
}

Eigen::Isometry3d KittiLoader::loadStaticTransformCameraFromVelodyne(const Path& calibration_folder)
{
    return loadStaticTransform(Path{calibration_folder} / Path{"calib_velo_to_cam.txt"});
}

std::vector<uint64_t> KittiLoader::loadTimestampsRaw(const Path& timestamp_path)
{
    std::ifstream is(timestamp_path);
    std::vector<uint64_t> timestamps;

    if (is.is_open())
    {
        std::string line;
        while (std::getline(is, line))
        {
            // remove fractional seconds
            std::vector<std::string> date_and_time = split(line, ' ');
            if (date_and_time.size() != 2)
                throw std::runtime_error("Unexpected time format: " + line);
            std::vector<std::string> time_and_fractional_seconds = split(date_and_time[1], '.');
            if (time_and_fractional_seconds.size() != 2)
                throw std::runtime_error("Unexpected time format: " + date_and_time[1]);
            std::string datetime_without_fractional_seconds = date_and_time[0] + " " + time_and_fractional_seconds[0];

            // parse string (without fractional seconds) and convert it to timestamp since epoch
            std::tm tm{};
            std::istringstream ss(datetime_without_fractional_seconds);
            ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
            if (ss.fail())
                throw std::runtime_error("Unable to parse datetime string: " + datetime_without_fractional_seconds);
            std::time_t t = std::mktime(&tm);

            // add fractional seconds
            if (time_and_fractional_seconds[1].length() != 9)
                throw std::runtime_error("Fractional seconds are not in nanosecond resolution: " + line);
            uint64_t nanoseconds = t * 1000000000UL + std::stoul(time_and_fractional_seconds[1]);
            timestamps.push_back(nanoseconds);
        }
    }
    else
        throw std::runtime_error("File does not exist: " + timestamp_path.string());

    return timestamps;
}

std::vector<uint64_t> KittiLoader::loadTimestamps(const Path& timestamp_path, bool make_fake_absolute)
{
    std::ifstream is(timestamp_path);
    std::vector<uint64_t> timestamps;

    uint64_t fake_start_stamp = 0;
    if (make_fake_absolute)
    {
        auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
        fake_start_stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch).count();
    }

    if (is.is_open())
    {
        std::string line;
        while (std::getline(is, line))
        {
            double dt = std::stod(line);
            timestamps.push_back(fake_start_stamp + static_cast<uint64_t>(dt * 1000000000UL));
        }
    }
    else
        throw std::runtime_error("File does not exist: " + timestamp_path.string());

    return timestamps;
}

void KittiLoader::getStartEndTimestampsVelodyne(const std::vector<uint64_t>& timestamps_middle,
                                                std::vector<uint64_t>& timestamps_start,
                                                std::vector<uint64_t>& timestamps_end)
{
    timestamps_start.clear();
    timestamps_start.resize(timestamps_middle.size());
    timestamps_end.clear();
    timestamps_end.resize(timestamps_middle.size());
    for (int i = 0; i < timestamps_middle.size() - 1; i++)
    {
        timestamps_end[i] = (timestamps_middle[i] + timestamps_middle[i + 1]) / 2;
        timestamps_start[i + 1] = timestamps_end[i];
    }
    timestamps_start.front() = timestamps_middle.front() - 50000000UL; // -50ms
    timestamps_end.back() = timestamps_middle.back() + 50000000UL;     // +50ms
}

std::map<int, RawSequenceSubset> KittiLoader::getKittiOdometrySequenceToKittiRawMapping()
{
    // from readme.txt in https://s3.eu-central-1.amazonaws.com/avg-kitti/devkit_raw_data.zip
    std::map<int, RawSequenceSubset> map;
    map.insert({0, {"2011_10_03", "2011_10_03_drive_0027_sync", 0, 4540}});
    map.insert({1, {"2011_10_03", "2011_10_03_drive_0042_sync", 0, 1100}});
    map.insert({2, {"2011_10_03", "2011_10_03_drive_0034_sync", 0, 4660}});
    map.insert({3, {"2011_09_26", "2011_09_26_drive_0067_sync", 0, 800}});
    map.insert({4, {"2011_09_30", "2011_09_30_drive_0016_sync", 0, 270}});
    map.insert({5, {"2011_09_30", "2011_09_30_drive_0018_sync", 0, 2760}});
    map.insert({6, {"2011_09_30", "2011_09_30_drive_0020_sync", 0, 1100}});
    map.insert({7, {"2011_09_30", "2011_09_30_drive_0027_sync", 0, 1100}});
    map.insert({8, {"2011_09_30", "2011_09_30_drive_0028_sync", 1100, 5170}});
    map.insert({9, {"2011_09_30", "2011_09_30_drive_0033_sync", 0, 1590}});
    map.insert({10, {"2011_09_30", "2011_09_30_drive_0034_sync", 0, 1200}});
    return map;
}

std::map<uint16_t, std::string> KittiLoader::getSemanticKittiLabelNumericToLabelNameMapping()
{
    // from https://github.com/PRBonn/semantic-kitti-api/blob/master/config/semantic-kitti.yaml
    std::map<uint16_t, std::string> map;
    map.insert({0, "unlabeled"});
    map.insert({1, "outlier"});
    map.insert({10, "car"});
    map.insert({11, "bicycle"});
    map.insert({13, "bus"});
    map.insert({15, "motorcycle"});
    map.insert({16, "on-rails"});
    map.insert({18, "truck"});
    map.insert({20, "other-vehicle"});
    map.insert({30, "person"});
    map.insert({31, "bicyclist"});
    map.insert({32, "motorcyclist"});
    map.insert({40, "road"});
    map.insert({44, "parking"});
    map.insert({48, "sidewalk"});
    map.insert({49, "other-ground"});
    map.insert({50, "building"});
    map.insert({51, "fence"});
    map.insert({52, "other-structure"});
    map.insert({60, "lane-marking"});
    map.insert({70, "vegetation"});
    map.insert({71, "trunk"});
    map.insert({72, "terrain"});
    map.insert({80, "pole"});
    map.insert({81, "traffic-sign"});
    map.insert({99, "other-object"});
    map.insert({252, "moving-car"});
    map.insert({253, "moving-bicyclist"});
    map.insert({254, "moving-person"});
    map.insert({255, "moving-motorcyclist"});
    map.insert({256, "moving-on-rails"});
    map.insert({257, "moving-bus"});
    map.insert({258, "moving-truck"});
    map.insert({259, "moving-other-vehicle"});
    return map;
}

std::map<std::string, uint16_t> KittiLoader::getSemanticKittiLabelNameToLabelNumericMapping()
{
    auto map = getSemanticKittiLabelNumericToLabelNameMapping();
    std::map<std::string, uint16_t> map_inv;
    for (const auto& p : map)
        map_inv.insert({p.second, p.first});
    return map_inv;
}

std::vector<std::string> KittiLoader::split(const std::string& s, char delimiter)
{
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;

    while (std::getline(ss, item, delimiter))
        result.push_back(item);

    return result;
}

std::string KittiLoader::padWithZeros(int v, int number_of_digits)
{
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(number_of_digits) << v;
    return ss.str();
}

} // namespace continuous_clustering