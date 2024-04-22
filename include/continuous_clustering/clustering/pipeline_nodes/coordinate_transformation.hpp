#ifndef CONTINUOUS_CLUSTERING_COORDINATE_TRANSFORMATION_HPP
#define CONTINUOUS_CLUSTERING_COORDINATE_TRANSFORMATION_HPP

#include <Eigen/Geometry>

#include <continuous_clustering/clustering/pipeline_nodes/pipeline_node.hpp>

namespace continuous_clustering
{

enum TransformStatus
{
    TRANSFORM_ERROR,
    TRANSFORM_NOT_AVAILABLE_YET,
    TRANSFORM_SUCCESS
};

struct CoordinateTransformationResult
{
    int64_t column_index;

    Eigen::Isometry3d odom_frame_from_sensor_frame;
};

template<class InputType>
class CoordinateTransformation : public PipelineNode<InputType, CoordinateTransformationResult>
{
  public:
    explicit CoordinateTransformation(uint8_t num_threads = 1)
        : PipelineNode<InputType, CoordinateTransformationResult>(num_threads, "T"){};

    void processJob(InputType&& job)
    {
        if (!request_transform_odom_from_sensor_callback_)
        {
            std::cout << "Please set the callback with setRequestTransformOdomFromSensorCallback()" << std::endl;
            return;
        }

        column_indices_waiting_for_transform.push(job.column_index);
        while (!column_indices_waiting_for_transform.empty())
        {
            int64_t column_index = column_indices_waiting_for_transform.front(); // todo use normal job queue?
            int local_column_index = this->range_image->toLocalColumnIndex(column_index);

            // get timestamp for this column
            int64_t column_stamp = this->range_image->getColumnStamp(local_column_index);
            if (column_stamp == 0)
            {
                column_indices_waiting_for_transform.pop();
                continue;
            }

            TransformStatus status;
            auto tf = request_transform_odom_from_sensor_callback_(
                this->range_image->getColumnStamp(local_column_index), status);
            if (status == TRANSFORM_NOT_AVAILABLE_YET) // wait and try again
                break;
            column_indices_waiting_for_transform.pop();
            if (status ==
                TRANSFORM_ERROR) // firing is not transformable as it arrived before every transform -> discard
                continue;

            // iterate from top to bottom
            for (int row_index = 0; row_index < this->range_image->num_rows; row_index++)
            {
                // obtain point
                Point& point = this->range_image->getPoint(row_index, local_column_index);

                // transform point to odom coordinate system todo: avoid Eigen dependency just because of this?
                Eigen::Vector3d p{point.xyz.x, point.xyz.y, point.xyz.z};
                p = tf * p;
                point.xyz.x = static_cast<float>(p.x());
                point.xyz.y = static_cast<float>(p.y());
                point.xyz.z = static_cast<float>(p.z());
            }

            CoordinateTransformationResult next_job;
            next_job.column_index = job.column_index;
            next_job.odom_frame_from_sensor_frame = std::move(tf);
            this->passJobToNextNode(next_job);
        }
    }

    void setRequestTransformOdomFromSensorCallback(std::function<Eigen::Isometry3d(int64_t, TransformStatus&)> cb)
    {
        request_transform_odom_from_sensor_callback_ = std::move(cb);
    }

    void reset()
    {
        column_indices_waiting_for_transform = std::queue<int64_t>();
    }

  private:
    std::function<Eigen::Isometry3d(int64_t, TransformStatus&)> request_transform_odom_from_sensor_callback_;
    std::queue<int64_t> column_indices_waiting_for_transform{};
};

} // namespace continuous_clustering

#endif // CONTINUOUS_CLUSTERING_COORDINATE_TRANSFORMATION_HPP
