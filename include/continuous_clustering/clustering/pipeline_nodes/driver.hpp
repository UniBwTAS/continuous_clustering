#ifndef CONTINUOUS_CLUSTERING_DRIVER_HPP
#define CONTINUOUS_CLUSTERING_DRIVER_HPP

#include <functional>

#include <continuous_clustering/clustering/pipeline_nodes/pipeline_node.hpp>
#include <continuous_clustering/clustering/point_types.hpp>

namespace continuous_clustering
{

struct DriverResult
{
    RawPoints::ConstPtr firing;
};

template<class InputType>
class Driver : public PipelineNode<InputType, DriverResult> // todo: public inherit -> protected working?
{
  public:
    explicit Driver(uint8_t num_threads = 1) : PipelineNode<InputType, DriverResult>(num_threads, "D"){};

    void reset() override
    {
        firing_index = 0;
        prepareNewFiring();
    }

  protected:
    void publishCurrentFiringAndPrepareNewFiring()
    {
        current_firing->stamp = min_stamp + (max_stamp - min_stamp) / 2;
        this->passJobToNextNode({current_firing});
        firing_index++;
        prepareNewFiring();
    }

    void prepareNewFiring()
    {
        // reset min and max stamp
        min_stamp = std::numeric_limits<int64_t>::max();
        max_stamp = 0;

        current_firing.reset(new RawPoints());
        current_firing->points.resize(num_lasers, RawPoint());
    }

    inline void keepTrackOfMinAndMaxStamp(int64_t point_stamp)
    {
        if (point_stamp < min_stamp)
            min_stamp = point_stamp;
        if (point_stamp > max_stamp)
            max_stamp = point_stamp;
    }

  protected:
    RawPoints::Ptr current_firing;
    int num_lasers{};
    std::vector<float> inclination_angles{};
    int64_t firing_index{0};
    int64_t min_stamp{0};
    int64_t max_stamp{0};
};

} // namespace continuous_clustering

#endif // CONTINUOUS_CLUSTERING_DRIVER_HPP
