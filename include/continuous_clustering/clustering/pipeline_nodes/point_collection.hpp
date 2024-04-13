#ifndef CONTINUOUS_CLUSTERING_POINT_COLLECTION_HPP
#define CONTINUOUS_CLUSTERING_POINT_COLLECTION_HPP

#include <continuous_clustering/clustering/pipeline_nodes/pipeline_node.hpp>

namespace continuous_clustering
{

struct PointCollectionConfig
{
    bool use_last_point_for_cluster_stamp{false};
};

struct PointCollectionResult
{
    int64_t column_index;

    std::vector<int64_t> outdated_minimum_required_column_indices;
    std::vector<std::pair<std::shared_ptr<std::vector<Point>>, uint64_t>> clusters_with_stamp;
};

template<class InputType>
class PointCollection : public PipelineNode<InputType, PointCollectionResult>
{
  public:
    explicit PointCollection(uint8_t num_treads = 3)
        : PipelineNode<InputType, PointCollectionResult>(num_treads, "C"){};

    void processJob(InputType&& job)
    {
        // keep track of minimum stamp for this message
        uint64_t min_stamp_for_this_msg = std::numeric_limits<uint64_t>::max();

        // todo
        std::vector<int64_t> outdated_minimum_required_column_indices;
        outdated_minimum_required_column_indices.push_back(job.column_index);

        std::vector<std::pair<std::shared_ptr<std::vector<Point>>, uint64_t>> clusters_with_stamp;

        auto it_trees_per_finished_cluster = job.trees_per_finished_cluster.begin();
        for (uint64_t cluster_id : job.cluster_ids)
        {
            // prepare vector
            auto cluster_points = std::make_shared<std::vector<Point>>();
            cluster_points->reserve(20000); // todo

            // collect minimum and maximum stamp for this cluster
            uint64_t min_stamp_for_this_cluster = std::numeric_limits<uint64_t>::max();
            uint64_t max_stamp_for_this_cluster = 0;

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
                    Point& cur_point =
                        this->range_image->getPoint(cur_point_index.row_index, cur_point_index.local_column_index);
                    cur_point.id = cluster_id;
                    cluster_points->push_back(cur_point);
                    if (cur_point.stamp < min_stamp_for_this_cluster)
                        min_stamp_for_this_cluster = cur_point.stamp;
                    if (cur_point.stamp > max_stamp_for_this_cluster)
                        max_stamp_for_this_cluster = cur_point.stamp;

                    // iterate over the edges of the current vertex
                    for (const RangeImageIndex& child_point_index : cur_point.child_points)
                        points_to_visit_for_this_tree.push_back(child_point_index);
                }

                // tell the range image that there is no need to wait for this point tree anymore
                outdated_minimum_required_column_indices.push_back(cur_tree_root_index.column_index);
            }

            // keep track of minimum stamp for this message
            if (min_stamp_for_this_cluster < min_stamp_for_this_msg)
                min_stamp_for_this_msg = min_stamp_for_this_cluster;

            // publish points (TODO: make threshold configurable)
            if (cluster_points->size() > 20)
            {
                uint64_t stamp_cluster =
                    config.use_last_point_for_cluster_stamp ?
                        max_stamp_for_this_cluster :
                        min_stamp_for_this_cluster + (max_stamp_for_this_cluster - min_stamp_for_this_cluster) / 2;

                // save points for this cluster
                clusters_with_stamp.emplace_back(std::move(cluster_points), stamp_cluster);
            }

            it_trees_per_finished_cluster++;
        }

        PointCollectionResult next_job;
        next_job.column_index = job.column_index;
        next_job.outdated_minimum_required_column_indices = std::move(outdated_minimum_required_column_indices);
        next_job.clusters_with_stamp = std::move(clusters_with_stamp);
        this->passJobToNextNode(next_job);
    }

    void setConfig(const PointCollectionConfig& c)
    {
        config = c;
    }

  private:
    PointCollectionConfig config;
};

} // namespace continuous_clustering

#endif // CONTINUOUS_CLUSTERING_POINT_COLLECTION_HPP