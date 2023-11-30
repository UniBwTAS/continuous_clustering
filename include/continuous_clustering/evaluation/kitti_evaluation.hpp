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

struct EvaluationResultForFrame
{
    // ground point segmentation
    double tp{0}; // true positive
    double fn{0}; // false negative
    double fp{0}; // false positive
    double tn{0}; // true negative

    // clustering
    double over_segmentation_entropy{0};
    double under_segmentation_entropy{0};
};

namespace euclidean_clustering
{
// From: https://github.com/url-kaist/TRAVEL/issues/6#issuecomment-1817809650
const float MAX_DISTANCE = 1.0;
const int MIN_CLUSTER_SIZE = 10;
const int MAX_CLUSTER_SIZE = 300000;
} // namespace euclidean_clustering

class KittiEvaluation
{
  public:
    KittiEvaluation();
    void evaluate(std::vector<KittiSegmentationEvaluationPoint>& point_cloud, int sequence_index);
    void evaluateGroundPoints(std::vector<KittiSegmentationEvaluationPoint>& point_cloud,
                              EvaluationResultForFrame& result_for_frame) const;
    static void evaluateClusters(std::vector<KittiSegmentationEvaluationPoint>& point_cloud,
                                 EvaluationResultForFrame& result_for_frame);
    static inline void addPointForKey(std::map<uint32_t, std::vector<KittiSegmentationEvaluationPoint>>& map,
                                      uint32_t key,
                                      const KittiSegmentationEvaluationPoint& point);
    std::string generateEvaluationResults();
    static std::vector<KittiSegmentationEvaluationPoint> convertPointCloud(const std::vector<KittiPoint>& in);
    std::vector<uint16_t> generateEuclideanClusteringLabels(std::vector<KittiPoint>& points) const;

  private:
    static inline bool isSameCluster(const pcl::PointXYZINormal& p1, const pcl::PointXYZINormal& p2, float sqr_dist);
    static void generateEvaluationResultForSingleMetric(
        std::stringstream& ss,
        const std::vector<EvaluationResultForFrame>& results_for_sequence,
        const std::string& metric_name,
        const std::function<double(const EvaluationResultForFrame&)>& metric_calc_fn);
    static inline void calculateMeanAndStdDev(const std::vector<double>& data, double& mean, double& std_dev);

  private:
    // ground point evaluation
    uint16_t label_lane_marking{};
    uint16_t label_road{};
    uint16_t label_parking{};
    uint16_t label_sidewalk{};
    uint16_t label_other_ground{};
    uint16_t label_terrain{};
    uint16_t label_unlabeled{};

    std::map<int, std::vector<EvaluationResultForFrame>> evaluation_per_sequence;
};

} // namespace continuous_clustering

#endif