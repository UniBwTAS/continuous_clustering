#include <iostream>

#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <continuous_clustering/evaluation/kitti_evaluation.hpp>

namespace continuous_clustering
{

KittiEvaluation::KittiEvaluation()
{
    auto name_to_label = KittiLoader::getSemanticKittiLabelNameToLabelNumericMapping();

    // get ground labels
    label_lane_marking = name_to_label.find("lane-marking")->second;
    label_road = name_to_label.find("road")->second;
    label_parking = name_to_label.find("parking")->second;
    label_sidewalk = name_to_label.find("sidewalk")->second;
    label_other_ground = name_to_label.find("other-ground")->second;
    label_terrain = name_to_label.find("terrain")->second;

    // get "unlabeled" label
    label_unlabeled = name_to_label.find("unlabeled")->second;

    // insert one pseudo entry -1 which contains averaged evaluation for all sequences
    evaluation_per_sequence.insert({-1, {}});
}

void KittiEvaluation::evaluate(std::vector<KittiSegmentationEvaluationPoint>& point_cloud, int sequence_index)
{
    auto result_it = evaluation_per_sequence.find(sequence_index);
    if (result_it == evaluation_per_sequence.end())
        result_it = evaluation_per_sequence.insert({sequence_index, {}}).first;
    EvaluationResult& result_total = evaluation_per_sequence[-1]; // results for all sequences concatenated

    evaluateGroundPoints(point_cloud, result_it->second, result_total);
    evaluateClusters(point_cloud, result_it->second, result_total);

    result_it->second.total_num_frames++;
    result_total.total_num_frames++;
}

void KittiEvaluation::evaluateGroundPoints(std::vector<KittiSegmentationEvaluationPoint>& point_cloud,
                                           EvaluationResult& result_sequence,
                                           EvaluationResult& result_total) const
{
    for (auto& p : point_cloud)
    {
        if (p.point.semantic_label == label_unlabeled)
            continue;

        bool gt_says_ground = p.point.semantic_label == label_lane_marking || p.point.semantic_label == label_road ||
                              p.point.semantic_label == label_parking || p.point.semantic_label == label_sidewalk ||
                              p.point.semantic_label == label_other_ground || p.point.semantic_label == label_terrain;
        bool segmentation_says_ground = p.is_ground_point;

        if (gt_says_ground)
        {
            if (segmentation_says_ground)
            {
                p.ground_point_true_positive = true;
                result_sequence.total_number_of_true_positives++;
                result_total.total_number_of_true_positives++;
            }
            else
            {
                p.ground_point_false_negative = true;
                result_sequence.total_number_of_false_negatives++;
                result_total.total_number_of_false_negatives++;
            }
        }
        else
        {
            if (segmentation_says_ground)
            {
                p.ground_point_false_positive = true;
                result_sequence.total_number_of_false_positives++;
                result_total.total_number_of_false_positives++;
            }
            else
            {
                p.ground_point_true_negative = true;
                result_sequence.total_number_of_true_negatives++;
                result_total.total_number_of_true_negatives++;
            }
        }
    }
}
void KittiEvaluation::evaluateClusters(std::vector<KittiSegmentationEvaluationPoint>& point_cloud,
                                       EvaluationResult& result_sequence,
                                       EvaluationResult& result_total)
{
    // prepare data
    std::map<uint32_t, std::vector<KittiSegmentationEvaluationPoint>> map_ground_truth_label_to_points;
    std::map<uint32_t, std::vector<KittiSegmentationEvaluationPoint>> map_detection_label_to_points;
    for (const auto& p : point_cloud)
    {
        if (p.euclidean_clustering_label != 0)
            addPointForKey(map_ground_truth_label_to_points, p.euclidean_clustering_label, p);

        if (p.detection_label != 0)
            addPointForKey(map_detection_label_to_points, p.detection_label, p);
    }

    // evaluate over-segmentation
    for (const auto& gt : map_ground_truth_label_to_points)
    {
        std::map<uint32_t, std::vector<KittiSegmentationEvaluationPoint>> map;
        for (auto p : gt.second)
            addPointForKey(map, p.detection_label, p);

        using pair_type = decltype(map)::value_type;

        // calculate over-segmentation entropy (OSE)
        double ose = 0;
        for (const auto& det_label_to_points : map)
        {
            double frac =
                static_cast<double>(det_label_to_points.second.size()) / static_cast<double>(gt.second.size());
            ose -= frac * std::log(frac);
        }
        result_sequence.total_sum_ose += ose;
        result_total.total_sum_ose += ose;

        auto max = std::max_element(map.begin(),
                                    map.end(),
                                    [](const pair_type& p1, const pair_type& p2)
                                    { return p1.second.size() < p2.second.size(); });
    }

    // evaluate under-segmentation
    for (const auto& det : map_detection_label_to_points)
    {
        std::map<uint32_t, std::vector<KittiSegmentationEvaluationPoint>> map;
        for (auto p : det.second)
            addPointForKey(map, p.euclidean_clustering_label, p);

        // this detection hasn't any ground truth point in it -> ignore
        if (map.size() == 1 && map.begin()->first == 0)
            continue;

        if (!map.empty())
        {
            // calculate under-segmentation entropy (USE)
            double use = 0;
            for (const auto& gt_label_to_points : map)
            {
                double frac =
                    static_cast<double>(gt_label_to_points.second.size()) / static_cast<double>(det.second.size());
                use -= frac * std::log(frac);
            }
            result_sequence.total_sum_use += use;
            result_total.total_sum_use += use;
        }
    }
}

inline void KittiEvaluation::addPointForKey(std::map<uint32_t, std::vector<KittiSegmentationEvaluationPoint>>& map,
                                            uint32_t key,
                                            const KittiSegmentationEvaluationPoint& point)
{
    auto lb = map.lower_bound(key);
    if (lb != map.end() && !(map.key_comp()(key, lb->first)))
        lb->second.push_back(point);
    else
        map.insert(lb, {key, {point}});
}

void KittiEvaluation::printEvaluationResults()
{
    std::cout << "\n" << std::endl;
    for (const auto& entry : evaluation_per_sequence)
    {
        std::cout << "------------------------------------------------------------------------------" << std::endl;
        if (entry.first == -1)
            std::cout << "| Sequence: All combined" << std::endl;
        else
            std::cout << "| Sequence: " << entry.first << std::endl;
        std::cout << "----------------------------Ground Point Segmentation-------------------------" << std::endl;

        const auto& result = entry.second;

        double recall =
            static_cast<double>(result.total_number_of_true_positives) /
            static_cast<double>(result.total_number_of_true_positives + result.total_number_of_false_negatives);
        std::cout << "| Recall: " << recall << std::endl;

        double precision =
            static_cast<double>(result.total_number_of_true_positives) /
            static_cast<double>(result.total_number_of_true_positives + result.total_number_of_false_positives);
        std::cout << "| Precision: " << precision << std::endl;

        double accuracy =
            static_cast<double>(result.total_number_of_true_positives + result.total_number_of_true_negatives) /
            static_cast<double>(result.total_number_of_true_positives + result.total_number_of_true_negatives +
                                result.total_number_of_false_positives + result.total_number_of_false_negatives);
        std::cout << "| Accuracy: " << accuracy << std::endl;

        double f1_score =
            static_cast<double>(result.total_number_of_true_positives + result.total_number_of_true_positives) /
            static_cast<double>(result.total_number_of_true_positives + result.total_number_of_true_positives +
                                result.total_number_of_false_positives + result.total_number_of_false_negatives);
        std::cout << "| F1 score: " << f1_score << std::endl;

        std::cout << "-----------------------------------Clustering---------------------------------" << std::endl;

        std::cout << "| Over-Segmentation Entropy: " << result.total_sum_ose / result.total_num_frames << ", "
                  << result.total_sum_ose << ", " << result.total_num_frames << std::endl;

        std::cout << "| Under-Segmentation Entropy: " << result.total_sum_use / result.total_num_frames << ", "
                  << result.total_sum_use << ", " << result.total_num_frames << std::endl;
        std::cout << "------------------------------------------------------------------------------\n" << std::endl;
    }
}

std::vector<KittiSegmentationEvaluationPoint> KittiEvaluation::convertPointCloud(const std::vector<KittiPoint>& in)
{
    std::vector<KittiSegmentationEvaluationPoint> out(0);
    out.reserve(in.size());
    for (const auto& p_in : in)
        out.push_back({p_in, false, false, false, false, false, false, 0, 0});
    return out;
}

std::vector<uint16_t> KittiEvaluation::generateEuclideanClusteringLabels(std::vector<KittiPoint>& points) const
{
    // convert to pcl point cloud
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_pc{new pcl::PointCloud<pcl::PointXYZINormal>(points.size(), 1)};
    for (int i = 0; i < points.size(); i++)
    {
        const KittiPoint& p_in = points[i];
        pcl::PointXYZINormal& p_out = (*pcl_pc)[i];

        p_out.x = p_in.x;
        p_out.y = p_in.y;
        p_out.z = p_in.z;
        p_out.intensity = static_cast<float>(p_in.semantic_label);
        p_out.curvature = static_cast<float>(p_in.instance_label);
    }

    // perform clustering
    pcl::IndicesClusters points_per_cluster;
    pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cluster_extractor;
    cluster_extractor.setInputCloud(pcl_pc);
    cluster_extractor.setConditionFunction(&isSameCluster);
    cluster_extractor.setMinClusterSize(euclidean_clustering::MIN_CLUSTER_SIZE);
    cluster_extractor.setMaxClusterSize(euclidean_clustering::MAX_CLUSTER_SIZE);
    cluster_extractor.setClusterTolerance(euclidean_clustering::MAX_DISTANCE);
    cluster_extractor.segment(points_per_cluster);

    // generate labels from clustering result
    std::vector<uint16_t> generated_labels(points.size(), 0);
    uint16_t cluster_index = 1;
    for (const auto& points_in_same_cluster : points_per_cluster)
    {
        for (const auto& point_idx : points_in_same_cluster.indices)
        {
            uint16_t s = points[point_idx].semantic_label;
            if (s == label_lane_marking || s == label_road || s == label_parking || s == label_sidewalk ||
                s == label_other_ground || s == label_terrain || s == label_unlabeled)
                generated_labels[point_idx] = 0;
            else
                generated_labels[point_idx] = cluster_index;
        }
        cluster_index++;
    }

    return generated_labels;
}

bool KittiEvaluation::isSameCluster(const pcl::PointXYZINormal& p1, const pcl::PointXYZINormal& p2, float sqr_dist)
{
    // intensity: label, curvature: id
    return sqr_dist < euclidean_clustering::MAX_DISTANCE * euclidean_clustering::MAX_DISTANCE &&
           p1.curvature == p2.curvature && p1.intensity == p2.intensity;
}

} // namespace continuous_clustering