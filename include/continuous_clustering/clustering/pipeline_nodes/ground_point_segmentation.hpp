#ifndef CONTINUOUS_CLUSTERING_GROUND_POINT_SEGMENTATION_HPP
#define CONTINUOUS_CLUSTERING_GROUND_POINT_SEGMENTATION_HPP

#include <Eigen/Geometry>

#include <continuous_clustering/clustering/pipeline_nodes/pipeline_node.hpp>

namespace continuous_clustering
{

struct GroundPointSegmentationResult
{
    int64_t column_index;
};

struct EgoVehicleBoundingBox
{
    float height_ref_to_top{};
    float height_ref_to_bottom{};
    float length_ref_to_front{};
    float length_ref_to_back{};
    float width_ref_to_left{};
    float width_ref_to_right{};

    std::optional<Eigen::Isometry3d> transform_ref_from_sensor;
};

struct GroundPointSegmentationConfig
{
    // Ego
    EgoVehicleBoundingBox ego_bounding_box;

    // General
    float max_slope{0.2};
    float first_ring_as_ground_max_allowed_z_diff{0.4};
    float first_ring_as_ground_min_allowed_z_diff{-0.4};

    // General Advanced
    float last_ground_point_slope_higher_than{-0.1};
    float last_ground_point_distance_smaller_than{5.};
    float ground_because_close_to_last_certain_ground_max_z_diff{0.4};
    float ground_because_close_to_last_certain_ground_max_dist_diff{2.0};
    float obstacle_because_next_certain_obstacle_max_dist_diff{0.3};
};

template<class InputType>
class GroundPointSegmentation : public PipelineNode<InputType, GroundPointSegmentationResult>
{
  public:
    explicit GroundPointSegmentation(uint8_t num_treads = 1)
        : PipelineNode<InputType, GroundPointSegmentationResult>(num_treads, "G"){};

    void processJob(InputType&& job)
    {
        Eigen::Vector3d t = job.odom_frame_from_sensor_frame.translation();
        Point3D sensor_position_in_odom;
        sensor_position_in_odom.x = static_cast<float>(t.x());
        sensor_position_in_odom.y = static_cast<float>(t.y());
        sensor_position_in_odom.z = static_cast<float>(t.z());

        int local_column_index = this->range_image->toLocalColumnIndex(job.column_index);

        Eigen::Isometry3d& ref_from_sensor = *config.ego_bounding_box.transform_ref_from_sensor;
        Eigen::Isometry3d ego_robot_frame_from_odom_frame =
            ref_from_sensor * job.odom_frame_from_sensor_frame.inverse();
        float height_sensor_to_ground =
            -static_cast<float>(ref_from_sensor.translation().z()) + config.ego_bounding_box.height_ref_to_bottom;

        // iterate rows from bottom to top and find ground points
        bool first_obstacle_detected = false;
        bool first_point_found = false;
        Point3D last_ground_position_wrt_sensor{0, 0, height_sensor_to_ground};
        Point3D previous_position_wrt_sensor;
        uint8_t previous_label;

        for (int row_index = this->range_image->num_rows - 1; row_index >= 0; row_index--)
        {
            // obtain point
            Point& point = this->range_image->getPoint(row_index, local_column_index);

            // skip ignored points
            if (std::isnan(point.distance) || point.debug_label == LIGHTGRAY) // todo: magic number/label
                continue;

            const Point3D& current_position = point.xyz;

            // special handling for points on ego vehicle surface
            Eigen::Vector3d current_position_in_ego_robot_frame =
                ego_robot_frame_from_odom_frame *
                Eigen::Vector3d(current_position.x, current_position.y, current_position.z);
            const auto& b = config.ego_bounding_box;
            if (current_position_in_ego_robot_frame.x() < b.length_ref_to_front &&
                current_position_in_ego_robot_frame.x() > b.length_ref_to_back &&
                current_position_in_ego_robot_frame.y() < b.width_ref_to_left &&
                current_position_in_ego_robot_frame.y() > b.width_ref_to_right &&
                current_position_in_ego_robot_frame.z() < b.height_ref_to_top &&
                current_position_in_ego_robot_frame.z() > b.height_ref_to_bottom)
            {
                point.ground_point_label = GP_EGO_VEHICLE;
                point.debug_label = VIOLET;
                point.is_ignored = true;
                continue;
            }

            Point3D current_position_wrt_sensor = current_position - sensor_position_in_odom;

            // special handling first point outside the ego bounding box
            if (!first_point_found)
            {
                // now we found the first point outside the ego vehicle box
                first_point_found = true;
                float height_over_predicted_ground = current_position_wrt_sensor.z - height_sensor_to_ground;
                if (height_over_predicted_ground > config.first_ring_as_ground_min_allowed_z_diff &&
                    height_over_predicted_ground < config.first_ring_as_ground_max_allowed_z_diff)
                {
                    point.ground_point_label = GP_GROUND;
                    point.debug_label = GRAY;
                    last_ground_position_wrt_sensor = current_position_wrt_sensor;
                    first_obstacle_detected = false;
                }
                else
                {
                    point.ground_point_label = GP_OBSTACLE;
                    point.debug_label = ORANGE;
                    first_obstacle_detected = true;
                }
                previous_position_wrt_sensor = current_position_wrt_sensor;
                previous_label = point.debug_label;
                point.is_ignored = point.ground_point_label != GP_OBSTACLE;
                continue;
            }

            // calculate the slope w.r.t previous point
            Point2D current_position_wrt_sensor_2d = to2D(current_position_wrt_sensor);
            Point2D previous_position_wrt_sensor_2d = to2D(previous_position_wrt_sensor);
            Point2D previous_to_current = current_position_wrt_sensor_2d - previous_position_wrt_sensor_2d;
            float slope_to_prev = previous_to_current.y / previous_to_current.x;
            bool is_flat_wrt_prev = std::abs(slope_to_prev) < config.max_slope && previous_to_current.x > 0;
            is_flat_wrt_prev = is_flat_wrt_prev && previous_to_current.x < 5; // TODO: Magic number

            // calculate slope w.r.t. last seen (quite certain) ground point
            Point2D last_ground_position_wrt_sensor_2d = to2D(last_ground_position_wrt_sensor);
            Point2D last_ground_to_current = current_position_wrt_sensor_2d - last_ground_position_wrt_sensor_2d;
            float slope_to_last_ground = last_ground_to_current.y / last_ground_to_current.x;
            bool is_flat_wrt_last_ground =
                std::abs(slope_to_last_ground) < config.max_slope && last_ground_to_current.x > 0;

            // quite certain ground points
            if (!first_obstacle_detected && is_flat_wrt_prev)
            {
                point.ground_point_label = GP_GROUND;
                point.debug_label = GREEN;
            }
            else if (first_obstacle_detected && is_flat_wrt_prev && is_flat_wrt_last_ground)
            {
                point.ground_point_label = GP_GROUND;
                point.debug_label = YELLOWGREEN;
            }
            else if (std::abs(last_ground_to_current.x) <
                         config.ground_because_close_to_last_certain_ground_max_dist_diff &&
                     std::abs(last_ground_to_current.y) < config.ground_because_close_to_last_certain_ground_max_z_diff)
            {
                point.ground_point_label = GP_GROUND;
                point.debug_label = YELLOW;
            }

            // mark remaining points as obstacle
            if (point.ground_point_label != GP_GROUND)
            {
                point.ground_point_label = GP_OBSTACLE;
                point.debug_label = RED;

                // go down in the rows and mark very close points also as obstacle
                int prev_row_index = row_index + 1;
                while (prev_row_index < this->range_image->num_rows)
                {
                    Point& cur_point = this->range_image->getPoint(prev_row_index, local_column_index);
                    Point2D prev_position_wrt_sensor_2d = to2D(cur_point.xyz - sensor_position_in_odom);
                    if (cur_point.debug_label == YELLOW ||
                        (cur_point.ground_point_label == GP_GROUND &&
                         std::abs((current_position_wrt_sensor_2d - prev_position_wrt_sensor_2d).x) <
                             config.obstacle_because_next_certain_obstacle_max_dist_diff))
                    {
                        if (cur_point.ground_point_label == GP_GROUND)
                        {
                            cur_point.ground_point_label = GP_OBSTACLE;
                            cur_point.debug_label = DARKRED;
                        }
                        prev_row_index++;
                    }
                    else
                    {
                        break;
                    }
                }
            }

            // check whether we have ever seen an obstacle
            first_obstacle_detected |= point.ground_point_label == GP_OBSTACLE;

            // keep track of last (certain) ground point
            if (point.debug_label == GREEN || point.debug_label == YELLOWGREEN)
            {
                // only use current point as the new last ground point when it was plausible. On wet streets there are
                // often false points below the ground surface because of reflections. Therefore, we do not want the
                // slope to be too much going down. Furthermore, in this case often there is a larger distance jump.
                if (slope_to_prev > config.last_ground_point_slope_higher_than &&
                    std::abs(previous_to_current.x) < config.last_ground_point_distance_smaller_than &&
                    previous_label != YELLOW)
                {
                    last_ground_position_wrt_sensor = current_position_wrt_sensor;
                }
            }

            // only consider obstacle points
            if (point.ground_point_label != GP_OBSTACLE)
                point.is_ignored = true;

            // keep track of previous point
            previous_position_wrt_sensor = current_position_wrt_sensor;
            previous_label = point.debug_label;
        }
        this->passJobToNextNode({job.column_index});
    }

    void setConfig(const GroundPointSegmentationConfig& c)
    {
        config = c;
    }

    GroundPointSegmentationConfig& getConfig()
    {
        return config;
    }

    static inline Point2D to2D(const Point3D& p)
    {
        return {p.xy().length(), p.z};
    }

  private:
    GroundPointSegmentationConfig config;
};

} // namespace continuous_clustering

#endif // CONTINUOUS_CLUSTERING_GROUND_POINT_SEGMENTATION_HPP
