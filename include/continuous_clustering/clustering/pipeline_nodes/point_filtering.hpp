#ifndef CONTINUOUS_CLUSTERING_POINT_FILTERING_HPP
#define CONTINUOUS_CLUSTERING_POINT_FILTERING_HPP

#include <cmath>

#include <continuous_clustering/clustering/pipeline_nodes/pipeline_node.hpp>

namespace continuous_clustering
{

struct PointFilteringResult
{
    int64_t column_index;
};

struct PointFilteringConfig
{
    // in order to reduce work load in the following we can ignore every second point in chessboard pattern
    bool ignore_points_in_chessboard_pattern{true};

    // ignore points which can't be clustered because the inclination angle to the lower row is too large (avoids single
    // row clusters; this requires the maximum clustering distance threshold)
    bool ignore_points_with_too_big_inclination_angle_diff{true};
    float max_distance{0.7}; // todo: rename?

    // ignore points which are too close
    float min_distance_from_sensor_origin{1.f};

    // ignore points originating from fog
    bool fog_filtering_enabled{false};
    uint8_t fog_filtering_intensity_below{2};
    float fog_filtering_distance_below{18};
    float fog_filtering_inclination_above{-0.06};
};

template<class InputType>
class PointFiltering : public PipelineNode<InputType, PointFilteringResult>
{
  public:
    explicit PointFiltering(uint8_t num_threads = 1) : PipelineNode<InputType, PointFilteringResult>(num_threads, "F"){};

    void processJob(InputType&& job)
    {
        int local_column_index = this->range_image->toLocalColumnIndex(job.column_index);

        // iterate from bottom to top
        for (int row_index = this->range_image->num_rows - 1; row_index >= 0; row_index--)
        {
            // obtain point
            Point& point = this->range_image->getPoint(row_index, local_column_index);

            // by default this point should be not ignored for clustering (in the next steps)
            point.is_ignored = false;

            // skip NaN's
            if (std::isnan(point.distance))
            {
                point.is_ignored = true;
                continue;
            }

            // skip points which seem to be fog
            if (config.fog_filtering_enabled && point.intensity < config.fog_filtering_intensity_below &&
                point.distance < config.fog_filtering_distance_below &&
                point.inclination_angle > config.fog_filtering_inclination_above)
            {
                point.debug_label = LIGHTGRAY;
                point.is_ignored = true;
                continue;
            }

            // ignore this point if it is too close
            if (point.distance < config.min_distance_from_sensor_origin)
            {
                point.is_ignored = true;
                continue;
            }

            // ignore this point if the distance in combination with inclination diff can't be below distance threshold
            if (config.ignore_points_with_too_big_inclination_angle_diff &&
                row_index < (this->range_image->num_rows - 1) &&
                std::atan2(config.max_distance, point.distance) <
                    this->range_image->inclination_angles_between_lasers[row_index])
            {
                point.is_ignored = true;
                continue;
            }

            // ignore this points in a chessboard pattern
            if (config.ignore_points_in_chessboard_pattern)
            {
                bool column_even = point.global_column_index % 2 == 0;
                bool row_even = row_index % 2 == 0;
                if ((column_even && !row_even) || (!column_even && row_even))
                {
                    point.is_ignored = true;
                    continue;
                }
            }
        }

        this->passJobToNextNode({job.column_index});
    }

    bool setConfig(const PointFilteringConfig& c)
    {
        config = c;
        return false;
    }

  private:
    PointFilteringConfig config;
};

} // namespace continuous_clustering

#endif // CONTINUOUS_CLUSTERING_POINT_FILTERING_HPP
