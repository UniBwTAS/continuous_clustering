#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <continuous_clustering/evaluation/kitti_evaluation.hpp>
#include <continuous_clustering/evaluation/kitti_loader.hpp>
#include <continuous_clustering/utils/command_line_parser.hpp>
#include <continuous_clustering/utils/thread_pool.hpp>

#ifdef WITH_ROS
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher publisher;
#endif
bool with_ros = true;

using Path = std::filesystem::path;

using namespace continuous_clustering;
using namespace std::chrono_literals;

struct PointWithLabels
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY_8U;
    uint16_t semantic_label;
    uint16_t instance_label;
    uint16_t cluster_label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointWithLabels,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, semantic_label, semantic_label)(
        std::uint16_t, instance_label, instance_label)(std::uint16_t, cluster_label, cluster_label));

struct Job
{
    Path velodyne_folder;
    Path labels_folder;
    int sequence{};
    int frame{};
    int number_of_frames{};
    Path generated_labels_folder;
};

void processSingleFrame(Job&& job)
{
    // load point cloud
    KittiLoader loader;
    std::string point_cloud_filename = KittiLoader::padWithZeros(job.frame, 6) + ".bin";
    std::vector<KittiPoint> points = loader.loadPointCloud(job.velodyne_folder / Path{point_cloud_filename});

    // also load semantic kitti labels
    std::string label_filename = KittiLoader::padWithZeros(job.frame, 6) + ".label";
    loader.loadSemanticKittiLabels(job.labels_folder / Path{label_filename}, points);

    // euclidean clustering
    KittiEvaluation evaluation;
    std::vector<uint16_t> euclidean_clustering_labels = evaluation.generateEuclideanClusteringLabels(points);

    // write data
    Path out_filename(job.generated_labels_folder / Path{label_filename});
    std::ofstream out;
    out.open(out_filename, std::ios::out | std::ios::binary);
    for (int i = 0; i < points.size(); i++)
        out.write(reinterpret_cast<const char*>(&euclidean_clustering_labels[i]), sizeof(uint16_t));

#ifdef WITH_ROS
    if (with_ros && publisher.getNumSubscribers() > 0)
    {
        pcl::PointCloud<PointWithLabels>::Ptr pcl_debug(new pcl::PointCloud<PointWithLabels>(points.size(), 1));
        for (int i = 0; i < points.size(); i++)
        {
            pcl_debug->points[i].x = points[i].x;
            pcl_debug->points[i].y = points[i].y;
            pcl_debug->points[i].z = points[i].z;
            pcl_debug->points[i].semantic_label = points[i].semantic_label;
            pcl_debug->points[i].instance_label = points[i].instance_label;
            pcl_debug->points[i].cluster_label = euclidean_clustering_labels[i];
        }
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*pcl_debug, msg);
        msg.header.frame_id = "velo_link";
        msg.header.stamp = ros::Time::now();
        publisher.publish(msg);
    }
#endif

    std::cout << "SEQUENCE: " << job.sequence << ", FRAME: " << job.frame << "/" << job.number_of_frames << std::endl;
}

int main(int argc, char** argv)
{
    // parse command line arguments
    utils::CommandLineParser parser(argc, argv);
    int num_threads = std::stoi(parser.getValueForArgument("--num-threads", "1"));
    with_ros = !parser.argumentExists("--no-ros");

#ifdef WITH_ROS
    if(with_ros)
    {
        ros::init(argc, argv, "gt_label_generator_tool");
        ros::NodeHandle nh;
        publisher = nh.advertise<sensor_msgs::PointCloud2>("gt_label_generator_tool/debug", 5);
    }
#endif

    // check if there are unknown arguments left
    for (const std::string& token : parser.getRemainingArgs())
        if (!token.empty() && token[0] == '-')
            throw std::runtime_error("Unknown argument: " + token);

    // get root folder and generate list of sequences from command line parameters
    std::vector<std::string> sequences;
    if (parser.getRemainingArgs().size() == 1)
    {
        std::cout << "Generating ground truth labels for all sequences in: " << parser.getRemainingArgs().front()
                  << std::endl;
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
    else if (parser.getRemainingArgs().size() > 1)
    {
        auto it = parser.getRemainingArgs().begin();
        std::cout << "Generating ground truth labels for the sequences: " << *it++ << std::endl;
        while (it != parser.getRemainingArgs().end())
        {
            std::string const sequence_folder{*it};
            std::cout << "\t" << sequence_folder << std::endl;
            sequences.push_back(sequence_folder);
            ++it;
        }
        std::cout << "\n" << std::endl;
    }
    else
    {
        std::cout << "You didn't pass the required command line arguments. Please pass the following command line "
                     "arguments:\n\n\t"
                     "To generate labels for all sequences in a provided data directory:\n\t\t"
                     "./gt_label_generator_tool [--num-threads N] <DATA_DIR>\n\n\t"
                     "To generate labels for specific sequences:\n\t\t"
                     "./gt_label_generator_tool <DATA_DIR> [--num-threads N] <SEQUENCE_1> <SEQUENCE_2> <SEQUENCE_N>"
                  << std::endl;
        return -1;
    }
    Path root_folder{argv[1]};

    // initialize thread pool
    continuous_clustering::ThreadPool<Job> thread_pool("", true);
    thread_pool.init([](Job&& job) { processSingleFrame(std::forward<Job>(job)); }, num_threads);

    // iterate over all frames in specified sequences
    for (const std::string& sequence : sequences)
    {
        // sequence path
        int sequence_index = std::stoi(sequence);
        Path sequence_folder{root_folder / Path(KittiLoader::padWithZeros(sequence_index, 2))};

        // velodyne path
        Path velodyne_folder{sequence_folder / Path{"velodyne"}};
        auto number_of_frames = static_cast<int>(
            std::distance(std::filesystem::directory_iterator{velodyne_folder}, std::filesystem::directory_iterator{}));

        // labels path
        Path labels_folder{sequence_folder / Path{"labels"}};
        if (!std::filesystem::exists(labels_folder))
        {
            std::cout << "Skip sequence as it does not contain ground truth labels: " << sequence_index << std::endl;
            continue;
        }

        Path generated_labels_folder{sequence_folder / Path("labels_euclidean_clustering")};
        if (!std::filesystem::is_directory(generated_labels_folder) ||
            !std::filesystem::exists(generated_labels_folder))
            std::filesystem::create_directory(generated_labels_folder);

        for (int frame_index = 0; frame_index < number_of_frames; ++frame_index)
        {
            Job j;
            j.velodyne_folder = velodyne_folder;
            j.labels_folder = labels_folder;
            j.sequence = sequence_index;
            j.frame = frame_index;
            j.number_of_frames = number_of_frames;
            j.generated_labels_folder = generated_labels_folder;
            thread_pool.enqueue(std::move(j));
        }
    }

#ifdef WITH_ROS
    while (thread_pool.getNumberOfUnprocessedJobs() > 0 && (!with_ros || ros::ok()))
        std::this_thread::sleep_for(1s);
#else
    thread_pool.join();
#endif

    return 0;
}