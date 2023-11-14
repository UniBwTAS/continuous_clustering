#ifndef CONTINUOUS_CLUSTERING_SENSOR_INPUT_HPP
#define CONTINUOUS_CLUSTERING_SENSOR_INPUT_HPP

#include <continuous_clustering/clustering/point_types.hpp>

namespace continuous_clustering
{

class SensorInput
{
  public:
    virtual void subscribe() = 0;
    virtual size_t dataCount() = 0;

    virtual void reset()
    {
        firing_index = 0;
        prepareNewFiring();
    };

    void addOnNewFiringCallback(const std::function<void(const RawPoints::ConstPtr&)>& c)
    {
        callback = c;
    }

  protected:
    void publishCurrentFiringAndPrepareNewFiring()
    {
        if (callback)
        {
            current_firing->stamp = min_stamp + (max_stamp - min_stamp) / 2;
            callback(current_firing);
        }
        firing_index++;
        prepareNewFiring();
    }

    void prepareNewFiring()
    {
        // reset min and max stamp
        min_stamp = std::numeric_limits<uint64_t>::max();
        max_stamp = 0;

        current_firing.reset(new RawPoints());
        current_firing->points.resize(num_lasers);
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
    std::function<void(const RawPoints::ConstPtr&)> callback;
    int num_lasers{};
    uint64_t firing_index{0};
    uint64_t min_stamp{0};
    uint64_t max_stamp{0};
};

} // namespace continuous_clustering

#endif
