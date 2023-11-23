#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <continuous_clustering/clustering/continuous_clustering.hpp>
#include <continuous_clustering/evaluation/kitti_evaluation.hpp>
#include <continuous_clustering/evaluation/kitti_loader.hpp>
#include <continuous_clustering/utils/command_line_parser.hpp>

#ifdef WITH_ROS
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <visualization_msgs/Marker.h>

#include <continuous_clustering/ros/ros_utils.hpp>
#endif

using Path = std::filesystem::path;

using namespace continuous_clustering;
using namespace std::chrono_literals;

class KittiDemo
{
  private:
    static inline RawPoints::Ptr makePseudoFiringFromRangeImageColumn(const std::vector<KittiPoint>& range_image,
                                                                      uint64_t start_stamp,
                                                                      uint64_t end_stamp,
                                                                      int column_index,
                                                                      int sequence_index,
                                                                      int frame_index)
    {
        RawPoints::Ptr firing(new RawPoints);

        // calculate approximate timestamp of this firing
        double elapsed_ratio = static_cast<double>(column_index) / (KittiLoader::RANGE_IMAGE_WIDTH - 1);
        double elapsed_time = static_cast<double>(end_stamp - start_stamp) * elapsed_ratio;
        firing->stamp = start_stamp + static_cast<uint64_t>(elapsed_time);

        // fill points
        firing->points.resize(KittiLoader::RANGE_IMAGE_HEIGHT);
        for (int row_index = 0; row_index < KittiLoader::RANGE_IMAGE_HEIGHT; row_index++)
        {
            uint32_t flattened_index = KittiLoader::RANGE_IMAGE_WIDTH * row_index + column_index;
            const KittiPoint& kitti_point = range_image[flattened_index];

            firing->points[row_index].stamp = firing->stamp;
            firing->points[row_index].firing_index = column_index;
            firing->points[row_index].x = kitti_point.x;
            firing->points[row_index].y = kitti_point.y;
            firing->points[row_index].z = kitti_point.z;
            firing->points[row_index].intensity = static_cast<uint8_t>(kitti_point.i * 255);

            // encode sequence, frame, and point index in pointcloud into one value in order to be able to obtain the
            // corresponding ground truth label for this point (also other subscribers in other nodes etc.)
            firing->points[row_index].globally_unique_point_index =
                (static_cast<uint64_t>(sequence_index) << 48) | (static_cast<uint64_t>(frame_index) << 32) |
                static_cast<uint64_t>(kitti_point.original_kitti_index);
        }

        return firing;
    }

    void evaluatePreviousFrame()
    {
        std::cout << "EVALUATE FRAME: " << current_sequence_index << ", " << previous_frame_index << std::endl;

        auto it = map_frame_to_point_cloud.find({current_sequence_index, previous_frame_index});
        evaluation.evaluate(it->second, current_sequence_index);
#ifdef WITH_ROS
        if (!disable_ros_publishers && pub_evaluation.getNumSubscribers() > 0)
            pub_evaluation.publish(evaluationToPointCloud(it->second));
#endif
        map_frame_to_point_cloud.erase(it);
        previous_frame_index++;
    }

    void addColumnAndEvaluateFrameIfCompleted(const ContinuousClustering& clustering,
                                              int64_t from_global_column_index,
                                              int64_t to_global_column_index)
    {
        int num_columns_to_publish = static_cast<int>(to_global_column_index - from_global_column_index) + 1;

        // iterate over the points in finished columns
        for (int relative_column_index = 0; relative_column_index < num_columns_to_publish; ++relative_column_index)
        {
            // get local column index from global column index
            int ring_buffer_local_column_index = static_cast<int>((from_global_column_index + relative_column_index) %
                                                                  clustering.ring_buffer_max_columns);

            // variables to check if a frame is finished
            bool new_frame = false;

            for (int row_index = 0; row_index < clustering.num_rows_; ++row_index)
            {
                // get processed point
                const Point& point =
                    clustering.range_image_[ring_buffer_local_column_index * clustering.num_rows_ + row_index];

                // check if cell in range image contains point
                if (point.globally_unique_point_index != static_cast<uint64_t>(-1))
                {
                    // get meta info from current point
                    uint16_t sequence_index = (point.globally_unique_point_index >> 48) & 0xFFFF;
                    uint16_t frame_index = (point.globally_unique_point_index >> 32) & 0xFFFF;
                    uint32_t kitti_point_index = point.globally_unique_point_index & 0xFFFFFFFF;

                    // check if we have a new frame
                    if (frame_index < previous_frame_index)
                        throw std::runtime_error("Found a point belonging to a frame that was already evaluated!");
                    else if (frame_index > previous_frame_index + 1)
                        throw std::runtime_error("Found a point whose frame is too far ahead!");
                    else if (frame_index == previous_frame_index + 1)
                        new_frame = true;

                    // add detection label to point in current ground truth point cloud
                    auto it = map_frame_to_point_cloud.find({sequence_index, frame_index});
                    KittiSegmentationEvaluationPoint& evaluation_point = it->second[kitti_point_index];
                    evaluation_point.is_ground_point = (point.ground_point_label == GP_GROUND);
                    evaluation_point.detection_label = point.id;
                    evaluation_point.has_corresponding_point_in_detection_point_cloud = true;
                }
            }

            // evaluate previous frame
            if (new_frame)
                evaluatePreviousFrame();
        }
    }

  public:
    void run(const Path& root_folder, const std::vector<std::string>& sequences)
    {
#ifdef WITH_ROS
        ros::NodeHandle nh;
        tf2_ros::TransformBroadcaster pub_tf;
        ros::Publisher pub_clock = nh.advertise<rosgraph_msgs::Clock>("/clock", 100);
        ros::Publisher pub_ego_robot_bbox = nh.advertise<visualization_msgs::Marker>("/ego_robot_bounding_box", 1);
        ros::Publisher pub_firing =
            nh.advertise<sensor_msgs::PointCloud2>("/perception/detections/lidar_roof/cluster/raw_firings", 1000);
        ros::Publisher pub_column_ground = nh.advertise<sensor_msgs::PointCloud2>(
            "/perception/detections/lidar_roof/cluster/continuous_ground_point_segmentation", 1000);
        ros::Publisher pub_column_cluster = nh.advertise<sensor_msgs::PointCloud2>(
            "/perception/detections/lidar_roof/cluster/continuous_instance_segmentation", 1000);
        ros::Publisher pub_cluster = nh.advertise<sensor_msgs::PointCloud2>(
            "/perception/detections/lidar_roof/cluster/continuous_clusters", 1000);
        pub_evaluation = nh.advertise<sensor_msgs::PointCloud2>("evaluation", 1000);
#endif

        // iterate over all frames in specified sequences
        KittiLoader kitti_loader;
        for (const std::string& sequence : sequences)
        {
            // sequence path
            uint16_t sequence_index = std::stoi(sequence);
            Path sequence_folder{root_folder / Path(KittiLoader::padWithZeros(sequence_index, 2))};
            std::cout << "RUN SEQUENCE: " << sequence_index << std::endl;

            // velodyne path (and path to ground truth labels)
            Path velodyne_folder{sequence_folder / Path{"velodyne"}};
            Path labels_folder{sequence_folder / Path{"labels"}};
            Path euclidean_labels_folder{sequence_folder / Path{"labels_euclidean_clustering"}}; // See "TRAVEL" paper

            // load velodyne timestamps
            auto timestamps_velodyne_middle = KittiLoader::loadTimestamps(sequence_folder / Path{"times.txt"}, true);
            std::vector<uint64_t> timestamps_velodyne_start;
            std::vector<uint64_t> timestamps_velodyne_end;
            KittiLoader::getStartEndTimestampsVelodyne(
                timestamps_velodyne_middle, timestamps_velodyne_start, timestamps_velodyne_end);

            // load all calibrations
            Path calib_path{sequence_folder / Path{"calib.txt"}};
            Eigen::Isometry3d tf_cam0_from_velodyne;
            Eigen::Affine3d projection_matrix_cam0;
            Eigen::Affine3d projection_matrix_cam1;
            Eigen::Affine3d projection_matrix_cam2;
            Eigen::Affine3d projection_matrix_cam3;
            kitti_loader.getStaticTransformAndProjectionMatrices(calib_path,
                                                                 tf_cam0_from_velodyne,
                                                                 projection_matrix_cam0,
                                                                 projection_matrix_cam1,
                                                                 projection_matrix_cam2,
                                                                 projection_matrix_cam3);

            // load all transforms odom from velodyne
            Path poses_path{sequence_folder / Path{"poses.txt"}};
            auto transforms_odom_from_velodyne =
                kitti_loader.getAllDynamicTransforms(poses_path, timestamps_velodyne_middle, tf_cam0_from_velodyne);

            // init clustering library
            ContinuousClustering clustering;

            // init some other configs
            Configuration config;
            config.range_image.num_columns = 2200;
            config.clustering.ignore_points_in_chessboard_pattern = false; // TODO
            config.clustering.max_distance = 0.5;
            config.ground_segmentation.height_ref_to_maximum_ = 0.5; // TODO: explain values
            config.ground_segmentation.height_ref_to_ground_ = -1.7;
            config.ground_segmentation.length_ref_to_front_end_ = 3;
            config.ground_segmentation.length_ref_to_rear_end_ = -3;
            config.ground_segmentation.width_ref_to_left_mirror_ = 1.5;
            config.ground_segmentation.width_ref_to_right_mirror_ = -1.5;
            clustering.setConfiguration(config);
            clustering.reset(64, true);
            clustering.setTransformRobotFrameFromSensorFrame(Eigen::Isometry3d::Identity());

            // add callbacks
            clustering.setFinishedColumnCallback(
                [&](int64_t from_global_column_index, int64_t to_global_column_index, bool ground_points_only)
                {
#ifdef WITH_ROS
                    // publish column in odom frame
                    ros::Publisher* pub = ground_points_only ? &pub_column_ground : &pub_column_cluster;
                    if (!disable_ros_publishers && pub->getNumSubscribers() > 0)
                    {
                        auto msg =
                            columnToPointCloud(clustering, from_global_column_index, to_global_column_index, "odom");
                        if (msg)
                            pub->publish(msg);
                    }
#endif

                    // evaluation
                    if (evaluate_current_sequence && !ground_points_only)
                        addColumnAndEvaluateFrameIfCompleted(
                            clustering, from_global_column_index, to_global_column_index);
                });

#ifdef WITH_ROS
            clustering.setFinishedClusterCallback(
                [&](const std::vector<Point>& cluster_points, uint64_t stamp_cluster)
                {
                    if (disable_ros_publishers || pub_cluster.getNumSubscribers() == 0)
                        return;

                    // publish cluster in odom frame
                    pub_cluster.publish(
                        clusterToPointCloud(cluster_points, clustering.num_rows_, stamp_cluster, "odom"));
                });
#endif

            // info required for evaluation
            current_sequence_index = sequence_index;
            previous_frame_index = 0;
            if (std::filesystem::exists(labels_folder))
            {
                std::cout << "Labels were found -> Evaluate this sequence" << std::endl;
                evaluate_current_sequence = true;
            }
            else
            {
                std::cout << "Labels were not found -> Don't evaluate this sequence" << std::endl;
                evaluate_current_sequence = false;
                if (skip_sequences_without_label)
                    continue;
            }

            // iterate over individual frames (i.e. full LiDAR rotations)
            auto num_frames = static_cast<uint16_t>(timestamps_velodyne_middle.size());
            for (uint16_t frame_index = 0; frame_index < num_frames; ++frame_index)
            {
                std::cout << "RUN SEQUENCE: " << sequence_index << ", FRAME: " << frame_index << std::endl;

                // load point cloud (with ground truth)
                std::string point_cloud_filename = KittiLoader::padWithZeros(frame_index, 6) + ".bin";
                std::vector<KittiPoint> points =
                    kitti_loader.loadPointCloud(velodyne_folder / Path{point_cloud_filename});

                // check weather there is ground truth for this sequence available
                if (evaluate_current_sequence)
                {
                    // also load semantic kitti labels (for ground point segmentation evaluation)
                    std::string label_filename = KittiLoader::padWithZeros(frame_index, 6) + ".label";
                    kitti_loader.loadSemanticKittiLabels(labels_folder / Path{label_filename}, points);

                    // generate/load euclidean clustering labels for evaluation
                    std::vector<uint16_t> euclidean_clustering_labels;
                    if (enable_online_ground_truth)
                        euclidean_clustering_labels = evaluation.generateEuclideanClusteringLabels(points);
                    else
                        euclidean_clustering_labels = KittiLoader::loadFlattenedPointCloud<uint16_t>(
                            euclidean_labels_folder / Path{label_filename});

                    // convert it to a point cloud with additional point fields for evaluation and save it
                    // add euclidean clustering labels (for Over-/Under-Segmentation-Entropy evaluation, see "TRAVEL"
                    // paper)
                    std::vector<KittiSegmentationEvaluationPoint> pc_eval = KittiEvaluation::convertPointCloud(points);
                    for (int i = 0; i < euclidean_clustering_labels.size(); i++)
                        pc_eval[i].euclidean_clustering_label = euclidean_clustering_labels[i];
                    map_frame_to_point_cloud.insert({{sequence_index, frame_index}, std::move(pc_eval)});
                }

                // recover the laser indices (in order to know the row index for the range image)
                kitti_loader.recoverLaserIndices(points);

                // undo ego motion correction. It is possible to omit this step. However, the number of cell
                // collisions during range image generation is greatly reduced. Furthermore, the jump in the rings
                // at the back of the ego vehicle is almost removed (not completely because there are some
                // reconstruction errors) we want the points to be as raw as possible
                kitti_loader.undoEgoMotionCorrection(points,
                                                     timestamps_velodyne_start[frame_index],
                                                     timestamps_velodyne_end[frame_index],
                                                     transforms_odom_from_velodyne[frame_index].pose,
                                                     transforms_odom_from_velodyne);

                // generate (flattened) range image from point cloud (height corresponds to number of lasers)
                std::vector<KittiPoint> range_image = kitti_loader.generateRangeImage(points);

#ifdef WITH_ROS
                // publish ego robot as marker
                if (!disable_ros_publishers)
                    publish_ego_robot_bounding_box(
                        timestamps_velodyne_start[frame_index], pub_ego_robot_bbox, config.ground_segmentation);
#endif

                // publish individual firings (here column and firing is the same as we pretend that the lasers
                // of one firing have the same azimuth angle, this is not the case for real velodyne sensors)
                for (int column_index = 0; column_index < KittiLoader::RANGE_IMAGE_WIDTH; column_index++)
                {
                    // build firing from range image column, convert it to a ROS message and publish it
                    auto firing = makePseudoFiringFromRangeImageColumn(range_image,
                                                                       timestamps_velodyne_start[frame_index],
                                                                       timestamps_velodyne_end[frame_index],
                                                                       column_index,
                                                                       sequence_index,
                                                                       frame_index);

#ifdef WITH_ROS
                    if (!disable_ros_publishers && pub_firing.getNumSubscribers() > 0)
                        pub_firing.publish(firingToPointCloud(firing, "velo_link"));
#endif

                    // get pose of sensor in odometry frame at this firing
                    Eigen::Isometry3d odom_from_velodyne =
                        kitti_loader.interpolate(transforms_odom_from_velodyne, firing->stamp).pose;

#ifdef WITH_ROS
                    // publish clock
                    if (!disable_ros_publishers)
                        publish_clock(pub_clock, firing->stamp);

                    // publish this pose every X-th column (not too often)
                    if (!disable_ros_publishers && column_index % 200 == 0)
                        publish_tf(pub_tf, odom_from_velodyne, firing->stamp);
#endif

                    clustering.addFiring(firing, odom_from_velodyne);

                    if (delay_between_columns > 0)
                    {
#ifdef WITH_ROS
                        if (!nh.ok())
                            return;
#endif
                        std::this_thread::sleep_for(std::chrono::microseconds(delay_between_columns));
                    }
                }

#ifdef WITH_ROS
                if (!nh.ok())
                    return;
#endif
            }

            // also evaluate final frame
            if (evaluate_current_sequence)
                evaluatePreviousFrame();
        }

        evaluation.printEvaluationResults();
    }

  private:
    KittiEvaluation evaluation;
#ifdef WITH_ROS
    ros::Publisher pub_evaluation;
#endif

    // store point cloud (+ ground truth labels)
    std::map<std::pair<uint16_t, uint16_t>, std::vector<KittiSegmentationEvaluationPoint>> map_frame_to_point_cloud;

    // members to detect finished frames (full point clouds)
    int current_sequence_index{0};
    int previous_frame_index{0};
    bool evaluate_current_sequence{false};

  public:
    bool disable_ros_publishers{};
    bool enable_online_ground_truth{};
    bool skip_sequences_without_label{};
    int delay_between_columns{};
};

int main(int argc, char** argv)
{
#ifdef WITH_ROS
    ros::init(argc, argv, "kitti_demo");
#endif

    KittiDemo demo;

    // parse command line arguments
    utils::CommandLineParser parser(argc, argv);
    demo.disable_ros_publishers = parser.argumentExists("--disable-ros-publishers");
    demo.enable_online_ground_truth = parser.argumentExists("--enable-online-ground-truth");
    demo.skip_sequences_without_label = parser.argumentExists("--skip-sequences-without-labels");
    demo.delay_between_columns = std::stoi(parser.getValueForArgument("--delay-between-columns", "0"));

    // check if there are unknown arguments left
    for (const std::string& token : parser.getRemainingArgs())
        if (!token.empty() && token[0] == '-')
            throw std::runtime_error("Unknown argument: " + token);

    // get root folder and generate list of sequences from command line parameters
    std::vector<std::string> sequences;
    if (parser.getRemainingArgs().size() == 1)
    {
        std::cout << "Run all sequences in: " << parser.getRemainingArgs().front() << std::endl;
        for (const auto& entry : std::filesystem::directory_iterator(parser.getRemainingArgs().front()))
        {
            if (entry.is_directory())
            {
                std::string const sequence_folder{entry.path().filename()};
                sequences.push_back(sequence_folder);
            }
        }
        std::sort(sequences.begin(), sequences.end());
        for (const auto& entry : sequences)
            std::cout << "\t" << entry << std::endl;
        std::cout << "\n" << std::endl;
    }
    else
    {
        auto it = parser.getRemainingArgs().begin();
        std::cout << "Run sequences: " << *it++ << std::endl;
        while (it != parser.getRemainingArgs().end())
        {
            std::string const sequence_folder{*it};
            std::cout << "\t" << sequence_folder << std::endl;
            sequences.push_back(sequence_folder);
            ++it;
        }
        std::cout << "\n" << std::endl;
    }

    demo.run(Path{argv[1]}, sequences);

    return 0;
}