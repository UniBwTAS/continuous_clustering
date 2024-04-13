#ifndef CONTINUOUS_CLUSTERING_CLUSTER_DETECTION_HPP
#define CONTINUOUS_CLUSTERING_CLUSTER_DETECTION_HPP

#include <continuous_clustering/clustering/pipeline_nodes/pipeline_node.hpp>

namespace continuous_clustering
{

struct ClusterDetectionResult
{
    int64_t column_index;

    std::list<uint64_t> cluster_ids;
    std::list<std::list<RangeImageIndex>> trees_per_finished_cluster;
};

template<class InputType>
class ClusterDetection : public PipelineNode<InputType, ClusterDetectionResult>
{
  public:
    explicit ClusterDetection(uint8_t num_treads = 1)
        : PipelineNode<InputType, ClusterDetectionResult>(num_treads, "D"){};

    void processJob(InputType&& job)
    {
        // add new point trees to existing unfinished point trees
        unfinished_point_trees_.splice(unfinished_point_trees_.end(), std::move(job.new_unfinished_point_trees));

        // keep track of point trees per finished cluster
        std::list<std::list<RangeImageIndex>> trees_per_finished_cluster;
        std::list<uint64_t> finished_cluster_ids;

        // run breath-first search (BFS) on each unpublished point tree. If a BFS from a specific starting node
        // completes without finding an unfinished point tree, then these point trees form a finished cluster.
        std::list<RangeImageIndex> trees_collected_for_current_cluster;
        std::list<RangeImageIndex> trees_to_visit_for_current_cluster;
        for (RangeImageIndex& tree_root_index : unfinished_point_trees_)
        {
            Point& point_root =
                this->range_image->getPoint(tree_root_index.row_index, tree_root_index.local_column_index);
            if (point_root.visited_at_continuous_azimuth_angle == job.current_minimum_continuous_azimuth_angle)
                continue; // this point tree was already visited, no need to use it as start node for BFS
            trees_collected_for_current_cluster.clear();
            trees_to_visit_for_current_cluster.clear();
            trees_to_visit_for_current_cluster.emplace_back(tree_root_index);
            int64_t min_column_index = std::numeric_limits<int64_t>::max();
            int64_t max_column_index = 0;
            uint32_t cluster_num_points = 0;
            bool cluster_has_at_least_one_unfinished_tree = false;
            while (!trees_to_visit_for_current_cluster.empty())
            {
                RangeImageIndex cur_tree_root_index = trees_to_visit_for_current_cluster.front();
                trees_to_visit_for_current_cluster.pop_front();
                Point& cur_point_root =
                    this->range_image->getPoint(cur_tree_root_index.row_index, cur_tree_root_index.local_column_index);

                // The following condition is always false under normal circumstances. It can only be true when a
                // cluster in the current graph was forcibly finished (exceeds full rotation) and when there was a race
                // condition with the association. In this case we simply ignore that a link between current point tree
                // and the previous (forcibly finished) point tree was found
                if (cur_point_root.belongs_to_finished_cluster ||
                    cur_tree_root_index.column_index < this->range_image->start_column_index)
                    continue;

                // keep track of the column range occupied by this cluster (to detect clusters larger than a full
                // rotation)
                min_column_index = std::min(min_column_index, cur_point_root.global_column_index);
                max_column_index =
                    std::max(max_column_index, cur_point_root.global_column_index + cur_point_root.cluster_width);

                // check if cluster is finished because LiDAR rotated far enough away
                if (cur_point_root.finished_at_continuous_azimuth_angle > job.current_minimum_continuous_azimuth_angle)
                    cluster_has_at_least_one_unfinished_tree = true;

                // check if this point tree was already visited. This can happen because the undirected graph (where
                // each node represents a point tree) is traversed by breath-first search.
                if (cur_point_root.visited_at_continuous_azimuth_angle == job.current_minimum_continuous_azimuth_angle)
                    continue;

                // mark that this node (point tree) was already traversed. Instead of a boolean flag we use a
                // monotonically rising value that does not need to be cleared
                cur_point_root.visited_at_continuous_azimuth_angle = job.current_minimum_continuous_azimuth_angle;

                // add this point tree to the current cluster
                trees_collected_for_current_cluster.push_back(cur_tree_root_index);
                cluster_num_points += cur_point_root.tree_num_points;

                // iterate over the edges of the current vertex
                for (const RangeImageIndex& other_tree_root_index : cur_point_root.associated_trees)
                {
                    Point& other_point_root = this->range_image->getPoint(other_tree_root_index.row_index,
                                                                          other_tree_root_index.local_column_index);
                    // if neighbor point tree was not already visited then traverse it too
                    if (other_point_root.visited_at_continuous_azimuth_angle !=
                        job.current_minimum_continuous_azimuth_angle)
                        trees_to_visit_for_current_cluster.push_back(other_tree_root_index);
                }
            }

            // check if finished point trees together already exceed one rotation
            bool finished_point_trees_exceed_one_rotation = false;
            if (max_column_index - min_column_index >= this->range_image->num_columns_per_rotation)
            {
                std::cout << "Found a cluster exceeding one rotation: " << (max_column_index - min_column_index) << " ("
                          << min_column_index << ", " << max_column_index << ")" << std::endl;
                finished_point_trees_exceed_one_rotation = true;
            }

            // skip cluster if at least one point tree is not finished (but not if we already have a full rotation)
            if ((trees_collected_for_current_cluster.empty() || cluster_has_at_least_one_unfinished_tree) &&
                !finished_point_trees_exceed_one_rotation)
                continue;

            // we found an actually (or forcibly) finished cluster; now we mark its trees finished and prepare them for
            // publishing
            for (const RangeImageIndex& cur_tree_root_index : trees_collected_for_current_cluster)
            {
                // mark current tree root as finished
                Point& cur_point_root =
                    this->range_image->getPoint(cur_tree_root_index.row_index, cur_tree_root_index.local_column_index);
                cur_point_root.belongs_to_finished_cluster = true;
            }

            trees_per_finished_cluster.push_back(std::move(trees_collected_for_current_cluster));
            finished_cluster_ids.push_back(cluster_counter_++);
        }

        // erase finished trees belonging to finished clusters
        auto it_erase = unfinished_point_trees_.begin();
        while (it_erase != unfinished_point_trees_.end())
        {
            Point& cur_point_root = this->range_image->getPoint(it_erase->row_index, it_erase->local_column_index);
            if (cur_point_root.belongs_to_finished_cluster)
                it_erase = unfinished_point_trees_.erase(it_erase);
            else
                it_erase++;
        }

        // create job for new clusters
        ClusterDetectionResult next_job;
        next_job.column_index = job.column_index;
        next_job.cluster_ids = std::move(finished_cluster_ids);
        next_job.trees_per_finished_cluster = std::move(trees_per_finished_cluster);
        this->passJobToNextNode(next_job);
    }

    void reset()
    {
        unfinished_point_trees_.clear();
        cluster_counter_ = 0;
    }

  private:
    std::list<RangeImageIndex> unfinished_point_trees_;
    uint64_t cluster_counter_{1};
};

} // namespace continuous_clustering

#endif // CONTINUOUS_CLUSTERING_CLUSTER_DETECTION_HPP
