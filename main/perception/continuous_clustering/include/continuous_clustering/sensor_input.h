#ifndef STREAMING_CLUSTERING_SENSOR_INPUT_H
#define STREAMING_CLUSTERING_SENSOR_INPUT_H

namespace streaming_clustering
{

struct RawPoint
{
    float x{};
    float y{};
    float z{};
    uint64_t firing_index{};
    uint8_t intensity{};
    uint64_t stamp{};
};

struct RawPoints
{
    std::vector<RawPoint> points;

    typedef std::shared_ptr<RawPoints> Ptr;
    typedef std::shared_ptr<RawPoints const> ConstPtr;
};

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
            callback(current_firing);
        firing_index++;
        prepareNewFiring();
    }

    void prepareNewFiring()
    {
        current_firing.reset(new RawPoints());
        current_firing->points.resize(num_lasers);
    }

  protected:
    RawPoints::Ptr current_firing;
    std::function<void(const RawPoints::ConstPtr&)> callback;
    int num_lasers{};
    uint64_t firing_index{0};
};

} // namespace streaming_clustering

#endif // STREAMING_CLUSTERING_SENSOR_INPUT_H
