#ifndef CONTINUOUS_CLUSTERING_ASSOCIATION_HPP
#define CONTINUOUS_CLUSTERING_ASSOCIATION_HPP

#include <cmath>

#include <continuous_clustering/clustering/pipeline_nodes/pipeline_node.hpp>

namespace continuous_clustering
{

struct AssociationResult
{
    int64_t column_index; // todo: add also local_column_index

    std::list<RangeImageIndex> new_unfinished_point_trees;
    double current_minimum_continuous_azimuth_angle;
};

struct AssociationConfig
{
    float max_distance{0.7};
    int max_steps_in_row{20};
    int max_steps_in_column{20};
    bool stop_after_association_enabled{true};
    int stop_after_association_min_steps{1};
};

template<class InputType>
class Association : public PipelineNode<InputType, AssociationResult>
{
  public:
    explicit Association(uint8_t num_threads = 1) : PipelineNode<InputType, AssociationResult>(num_threads, "A"){};

    void processJob(InputType&& job)
    {
        // collect new point trees starting with this column
        std::list<RangeImageIndex> new_unfinished_point_trees{};

        // keep track of the current minimum azimuth angle of the current column
        double current_minimum_continuous_azimuth_angle = std::numeric_limits<double>::max();

        // get global & local column indices of current column
        int64_t column_index = job.column_index;
        int local_column_index = this->range_image->toLocalColumnIndex(column_index);

        // get lower bound we want to reserve in order to ensure that columns are not cleared during fov traversal
        int64_t minimum_column_index = job.column_index - config.max_steps_in_row;

        // ensure that no columns are cleared during traversal and that we do not iterate past the start of the range
        // image
        minimum_column_index = this->range_image->addMinimumRequiredStartColumnIndex(minimum_column_index, true);
        int minimum_local_column_index = this->range_image->toLocalColumnIndex(minimum_column_index);

        for (int row_index = 0; row_index < this->range_image->num_rows; row_index++)
        {
            // get current point
            Point& point = this->range_image->getPoint(row_index, local_column_index);

            // keep track of the current minimum continuous azimuth angle
            if (point.continuous_azimuth_angle < current_minimum_continuous_azimuth_angle)
                current_minimum_continuous_azimuth_angle = point.continuous_azimuth_angle;

            // check whether point should be ignored
            if (point.is_ignored)
                continue;

            // create index of this point
            RangeImageIndex index{static_cast<uint16_t>(row_index), column_index, local_column_index};

            // calculate minimum possible azimuth angle
            float max_angle_diff = std::asin(config.max_distance / point.distance);

            // traverse field of view
            traverseFieldOfView(point, max_angle_diff, minimum_local_column_index);

            // point could not be associated to any other point in field of view
            if (point.tree_root_.local_column_index == -1)
            {
                // this point could not be associated to any tree -> init new tree
                point.tree_root_ = index;
                point.tree_id = point.global_column_index * this->range_image->num_rows + point.row_index;

                // calculate finish time (will be increased when new points are associated to tree)
                point.finished_at_continuous_azimuth_angle = point.continuous_azimuth_angle + max_angle_diff;
                point.cluster_width = 1;

                // new tree has 1 point
                point.tree_num_points = 1;

                // create shortcut to this tree root
                new_unfinished_point_trees.push_back(index);
            }
        }

        // remove our column lock
        this->range_image->removeMinimumRequiredStartColumnIndex(minimum_column_index);

        std::vector<int64_t> minimum_required_column_indices;
        for (const auto& new_point_tree : new_unfinished_point_trees)
            minimum_required_column_indices.push_back(new_point_tree.column_index);
        minimum_required_column_indices.push_back(job.column_index);
        this->range_image->addMinimumRequiredStartColumnIndices(minimum_required_column_indices);

        // pass this job to the next thread
        AssociationResult next_job;
        next_job.column_index = job.column_index;
        next_job.new_unfinished_point_trees = std::move(new_unfinished_point_trees);
        next_job.current_minimum_continuous_azimuth_angle = current_minimum_continuous_azimuth_angle;
        this->passJobToNextNode(next_job);
    }

    void associatePointToPointTree(Point& point, Point& point_other, float max_angle_diff)
    {
        // this point is not associated to any point tree -> associate it
        // Furthermore, we also have to check that clusters are forcibly published when they span a
        // full rotation. For this we take two measures: (1) don't associate this point to the point
        // tree when it would cover more than one rotation; (2) don't associate this point to the
        // point tree, when its cluster is considered to be finished. Normally, the latter can not
        // happen because a cluster is only considered to be finished when no more point can be
        // associated (LiDAR rotated far enough away from a cluster's point trees). It can only
        // happen when a cluster is forcibly considered to be finished because it already spans a
        // full rotation. Then we do not want to associate more points to it.
        Point& point_root =
            this->range_image->getPoint(point_other.tree_root_.row_index, point_other.tree_root_.local_column_index);
        uint32_t new_cluster_width = point.global_column_index - point_root.global_column_index + 1;
        bool point_tree_is_smaller_than_one_rotation =
            new_cluster_width <= this->range_image->num_columns_per_rotation;          // (1)
        bool point_tree_is_finished_forcibly = point_root.belongs_to_finished_cluster; // (2)
        if (point_tree_is_smaller_than_one_rotation && !point_tree_is_finished_forcibly)
        {
            point.tree_root_ = point_other.tree_root_;
            point.tree_id = point_root.global_column_index * this->range_image->num_rows + point_root.row_index;
            point_other.child_points.emplace_back(
                static_cast<uint16_t>(point.row_index), point.global_column_index, point.local_column_index);

            // keep track of cluster width
            point_root.cluster_width = new_cluster_width;

            // new point was associated to tree -> check if finish time will be delayed
            point_root.finished_at_continuous_azimuth_angle = std::max(point_root.finished_at_continuous_azimuth_angle,
                                                                       point.continuous_azimuth_angle + max_angle_diff);
            point_root.tree_num_points++;
        }
    }

    void associatePointTreeToPointTree(const Point& point, const Point& point_other)
    {
        // tree root of this point must be different to the other point (because of check above
        // *1) so we must add a mutual link between the trees roots

        // get tree root of current and other point
        Point& point_root =
            this->range_image->getPoint(point.tree_root_.row_index, point.tree_root_.local_column_index);
        Point& point_root_other =
            this->range_image->getPoint(point_other.tree_root_.row_index, point_other.tree_root_.local_column_index);

        // similar to above (unassociated point is added to point tree) we also don't want to
        // introduce links between point trees when either of them was forcibly finished because its
        // corresponding cluster already covers a full rotation
        bool neither_cluster_was_forcibly_finished =
            !point_root.belongs_to_finished_cluster && !point_root_other.belongs_to_finished_cluster;
        if (neither_cluster_was_forcibly_finished)
        {
            // add mutual link
            point_root.associated_trees.insert(point_other.tree_root_);
            point_root_other.associated_trees.insert(point.tree_root_);
        }
    }

    void traverseFieldOfView(Point& point, float max_angle_diff, int minium_column_index)
    {
        // go left each column until azimuth angle difference gets too large
        int required_steps_back =
            static_cast<int>(std::ceil(max_angle_diff / this->range_image->azimuth_range_of_single_column));
        required_steps_back = std::min(required_steps_back, config.max_steps_in_row);
        int other_column_index = point.local_column_index;
        for (int num_steps_back = 0; num_steps_back <= required_steps_back; num_steps_back++)
        {
            for (int direction = -1; direction <= 1; direction += 2)
            {
                // do not go down in first column (these points are not associated to tree yet!)
                if (direction == 1 && num_steps_back == 0)
                    continue;

                // go up/down each row until the inclination angle difference gets too large
                int num_steps_vertical = direction == 1 || num_steps_back == 0 ? 1 : 0;
                int other_row_index =
                    direction == 1 || num_steps_back == 0 ? point.row_index + direction : point.row_index;
                while (other_row_index >= 0 && other_row_index < this->range_image->num_rows &&
                       num_steps_vertical <= config.max_steps_in_column)
                {
                    // get other point
                    Point& point_other = this->range_image->getPoint(other_row_index, other_column_index);

                    // count number of visited points for analyzing
                    point.number_of_visited_neighbors += 1;

                    // no cluster can be associated because the inclination angle diff gets too large
                    if (std::abs(point_other.inclination_angle - point.inclination_angle) > max_angle_diff)
                        break;

                    // if other point is ignored or has already the same tree root then do nothing (*1)
                    if (!point_other.is_ignored &&
                        (point.tree_root_.local_column_index == -1 || point_other.tree_root_ != point.tree_root_))
                    {
                        // if distance is small enough then try to associate to tree
                        if (checkClusteringCondition(point, point_other))
                        {
                            // check if point is already associated to any point tree
                            if (point.tree_root_.local_column_index == -1)
                                associatePointToPointTree(point, point_other, max_angle_diff);
                            else
                                associatePointTreeToPointTree(point, point_other);
                        }
                    }

                    // stop searching if point was already associated and minimum number of columns were processed
                    if (point.tree_root_.local_column_index != -1 && config.stop_after_association_enabled &&
                        num_steps_vertical >= config.stop_after_association_min_steps)
                        break;

                    other_row_index += direction;
                    num_steps_vertical++;
                }
            }

            // stop searching if point was already associated and minimum number of points were processed
            if (point.tree_root_.local_column_index != -1 && config.stop_after_association_enabled &&
                num_steps_back >= config.stop_after_association_min_steps)
                break;

            // stop searching if we are at the beginning of the ring buffer
            if (other_column_index == minium_column_index)
                break;

            other_column_index--;

            // jump to the end of the ring buffer
            if (other_column_index < 0)
                other_column_index += this->range_image->max_columns_ring_buffer;
        }
    }

    bool checkClusteringCondition(const Point& point, const Point& point_other) const
    {
        return (point.xyz - point_other.xyz).lengthSquared() < max_distance_squared;
    }

    bool setConfig(const AssociationConfig& c)
    {
        config = c;
        max_distance_squared = c.max_distance * c.max_distance;
        return false;
    }

  private:
    AssociationConfig config;
    float max_distance_squared{0.7 * 0.7};
};

} // namespace continuous_clustering

#endif // CONTINUOUS_CLUSTERING_ASSOCIATION_HPP
