#ifndef CONTINUOUS_CLUSTERING_KITTI_LOADER_HPP
#define CONTINUOUS_CLUSTERING_KITTI_LOADER_HPP

#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>

using Path = std::filesystem::path;

namespace continuous_clustering
{

// meta data
struct RawSequenceSubset
{
    std::string day_string;
    std::string sequence_string;
    int first_frame;
    int last_frame;
};

// lidar
struct KittiPoint
{
    // original point cloud
    float x{std::nanf("")};
    float y{std::nanf("")};
    float z{std::nanf("")};
    float i{std::nanf("")};

    // semantic kitti labels
    uint16_t semantic_label{0};
    uint16_t instance_label{0};

    // important to generate compact range image (will be row index in range image)
    uint8_t laser_index{0};

    // only used when in range images where the points are rearranged
    int32_t original_kitti_index{-1};
};

// oxts
struct Oxts
{
    int64_t stamp;
    double lat;
    double lon;
    double alt;
    double roll;
    double pitch;
    double yaw;
    double vf;
    double vl;
    double vu;
};

// camera
struct CameraCalibration
{
    Eigen::Vector2d S;
    Eigen::Matrix3d K;
    Eigen::Matrix<double, 5, 1> D;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    Eigen::Vector2d S_rect;
    Eigen::Matrix3d R_rect;
    Eigen::Matrix<double, 3, 4> P_rect;
};

// transforms
struct StampedPose
{
    int64_t stamp;
    Eigen::Isometry3d pose;
};

class KittiLoader
{
  public:
    static const int NUM_LASERS = 64;
    static const int RANGE_IMAGE_HEIGHT = NUM_LASERS;
    static const int RANGE_IMAGE_WIDTH = 2200; // max observed so far: 2172

  public:
    explicit KittiLoader();

    // LiDAR
    std::vector<KittiPoint> loadPointCloud(const Path& path);
    void loadSemanticKittiLabels(const Path& path, std::vector<KittiPoint>& points);
    template<typename T>
    static std::vector<T> loadFlattenedPointCloud(const Path& path)
    {
        // open file stream
        std::ifstream fs{path, std::ios::in | std::ios::binary | std::ios::ate};
        if (not fs.is_open())
            throw std::runtime_error("Unable to open file: " + path.string());

        // get file size
        int64_t number_of_bytes{fs.tellg()};
        if (number_of_bytes == -1 || number_of_bytes % sizeof(T) != 0)
            throw std::runtime_error("File seems to be corrupt: " + path.string());

        // prepare buffer with the correct size
        std::vector<T> flattened_points;
        flattened_points.resize(number_of_bytes / sizeof(T));

        // write bytes to buffer
        fs.seekg(0, std::ios::beg);
        fs.read(reinterpret_cast<char*>(flattened_points.data()), number_of_bytes);
        fs.close();

        return flattened_points;
    }
    void recoverLaserIndices(std::vector<KittiPoint>& points);
    std::vector<KittiPoint> generateRangeImage(const std::vector<KittiPoint>& unorganized_points,
                                               bool shift_cell_if_already_occupied = true);
    void undoEgoMotionCorrection(std::vector<KittiPoint>& corrected_points,
                                 int64_t rotation_start_stamp,
                                 int64_t rotation_end_stamp,
                                 const Eigen::Isometry3d& odom_from_velodyne_at_middle_of_rotation,
                                 const std::vector<StampedPose>& odom_from_velodyne);

    // Oxford/Poses
    Oxts loadSingleOxfordMeasurement(const Path& path);
    Eigen::Isometry3d convertOxfordMeasurementToPose(const Oxts& oxts, double scale);
    std::vector<StampedPose>
    getAllDynamicTransforms(const Path& path_oxford_folder,
                            int first_frame,
                            int last_frame,
                            const Eigen::Isometry3d& tf_oxford_from_x = Eigen::Isometry3d::Identity());
    std::vector<StampedPose> makeTransformsRelativeToFirstTransform(const std::vector<StampedPose>& transforms);
    StampedPose interpolate(const std::vector<StampedPose>& transforms, int64_t stamp);
    std::vector<StampedPose>
    getAllDynamicTransforms(const Path& path_poses_file,
                            const std::vector<int64_t>& timestamps = {},
                            const Eigen::Isometry3d& tf_cam0_from_x = Eigen::Isometry3d::Identity());
    void getStaticTransformAndProjectionMatrices(const Path& path_calib_file,
                                                 Eigen::Isometry3d& tf_cam0_from_velodyne,
                                                 Eigen::Affine3d& projection_matrix_cam0,
                                                 Eigen::Affine3d& projection_matrix_cam1,
                                                 Eigen::Affine3d& projection_matrix_cam2,
                                                 Eigen::Affine3d& projection_matrix_cam3);

    // Calibrations
    Eigen::Isometry3d loadStaticTransform(const Path& calibration_file);
    Eigen::Isometry3d loadStaticTransformVelodyneFromOxford(const Path& calibration_folder);
    Eigen::Isometry3d loadStaticTransformCameraFromVelodyne(const Path& calibration_folder);

    // Timing
    static std::vector<int64_t> loadTimestampsRaw(const Path& timestamp_path);
    static std::vector<int64_t> loadTimestamps(const Path& timestamp_path, bool make_fake_absolute);
    static void getStartEndTimestampsVelodyne(const std::vector<int64_t>& timestamps_middle,
                                              std::vector<int64_t>& timestamps_start,
                                              std::vector<int64_t>& timestamps_end);

    // Meta Data
    static std::map<int, RawSequenceSubset> getKittiOdometrySequenceToKittiRawMapping();
    static std::map<uint16_t, std::string> getSemanticKittiLabelNumericToLabelNameMapping();
    static std::map<std::string, uint16_t> getSemanticKittiLabelNameToLabelNumericMapping();

    // Utils
    static std::vector<std::string> split(const std::string& s, char delimiter);
    static std::string padWithZeros(int v, int number_of_digits);
};
} // namespace continuous_clustering

#endif