#ifndef CONTINUOUS_CLUSTERING_KITTI_EVALUATION_HPP
#define CONTINUOUS_CLUSTERING_KITTI_EVALUATION_HPP

#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <pcl/point_types.h>
#include <Eigen/Geometry>

#include <continuous_clustering/evaluation/kitti_loader.hpp>

namespace continuous_clustering
{

struct KittiSegmentationEvaluationPoint
{
    // original kitti point
    KittiPoint point;

    // this flag is set when there is a corresponding point in the clustering output
    bool has_corresponding_point_in_detection_point_cloud{false};

    // ground point segmentation
    bool is_ground_point{false};
    bool ground_point_true_positive{false};
    bool ground_point_false_negative{false};
    bool ground_point_false_positive{false};
    bool ground_point_true_negative{false};

    // clustering
    uint32_t euclidean_clustering_label{0};
    uint32_t detection_label{0};
};

struct EvaluationResult
{
    // ground point segmentation
    uint64_t total_number_of_true_positives{};
    uint64_t total_number_of_false_negatives{};
    uint64_t total_number_of_false_positives{};
    uint64_t total_number_of_true_negatives{};

    // clustering
    double total_sum_ose{0};
    double total_sum_use{0};
    int total_num_frames{0};
};

namespace euclidean_clustering
{
const float MAX_DISTANCE = 0.5; // TODO: find real value from TRAVEL paper
const int MIN_CLUSTER_SIZE = 1;
const int MAX_CLUSTER_SIZE = 99999999;
} // namespace euclidean_clustering

class KittiEvaluation
{
  public:
    KittiEvaluation();
    void evaluate(std::vector<KittiSegmentationEvaluationPoint>& point_cloud, int sequence_index);
    void evaluateGroundPoints(std::vector<KittiSegmentationEvaluationPoint>& point_cloud,
                              EvaluationResult& result_sequence,
                              EvaluationResult& result_total) const;
    static void evaluateClusters(std::vector<KittiSegmentationEvaluationPoint>& point_cloud,
                                 EvaluationResult& result_sequence,
                                 EvaluationResult& result_total);
    static inline void addPointForKey(std::map<uint32_t, std::vector<KittiSegmentationEvaluationPoint>>& map,
                                      uint32_t key,
                                      const KittiSegmentationEvaluationPoint& point);
    void printEvaluationResults();
    static std::vector<KittiSegmentationEvaluationPoint> convertPointCloud(const std::vector<KittiPoint>& in);
    std::vector<uint16_t> generateEuclideanClusteringLabels(std::vector<KittiPoint>& points) const;

  private:
    static inline bool isSameCluster(const pcl::PointXYZINormal& p1, const pcl::PointXYZINormal& p2, float sqr_dist);

  private:
    // ground point evaluation
    uint16_t label_lane_marking{};
    uint16_t label_road{};
    uint16_t label_parking{};
    uint16_t label_sidewalk{};
    uint16_t label_other_ground{};
    uint16_t label_terrain{};
    uint16_t label_unlabeled{};

    std::map<int, EvaluationResult> evaluation_per_sequence;
};

} // namespace continuous_clustering

#endif