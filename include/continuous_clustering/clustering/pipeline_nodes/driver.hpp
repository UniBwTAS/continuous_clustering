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

class Driver : PipelineNode<int, DriverResult> // int is just some dummy
{
  public:
    explicit Driver(uint8_t num_treads = 1) : PipelineNode<int, DriverResult>(num_treads, "D"){};

    virtual void subscribe() = 0;
    // virtual size_t dataCount() = 0;

    void reset() override
    {
        firing_index = 0;
        prepareNewFiring();
    }

  protected:
    void publishCurrentFiringAndPrepareNewFiring()
    {
        current_firing->stamp = min_stamp + (max_stamp - min_stamp) / 2;
        passJobToNextNode({current_firing});
        firing_index++;
        prepareNewFiring();
    }

    void prepareNewFiring()
    {
        // reset min and max stamp
        min_stamp = std::numeric_limits<uint64_t>::max();
        max_stamp = 0;

        current_firing.reset(new RawPoints());
        current_firing->points.resize(num_lasers, RawPoint());
    }

    inline void keepTrackOfMinAndMaxStamp(uint64_t point_stamp)
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
    uint64_t firing_index{0};
    uint64_t min_stamp{0};
    uint64_t max_stamp{0};
};

} // namespace continuous_clustering

#endif // CONTINUOUS_CLUSTERING_DRIVER_HPP
