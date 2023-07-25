#ifndef STREAMING_CLUSTERING_ROS_SENSOR_INPUT_H
#define STREAMING_CLUSTERING_ROS_SENSOR_INPUT_H

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <streaming_clustering/sensor_input.h>

using namespace std::chrono_literals;

namespace streaming_clustering
{

template<typename Message>
class RosSensorInput : public SensorInput
{
  public:
    RosSensorInput(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh(nh)
    {
    }

    void subscribe() override
    {
        sub = nh.subscribe(
            "raw_data", 1000, &RosSensorInput::onRawDataArrivedInternal, this, ros::TransportHints().tcpNoDelay(true));
    }

    void reset() override
    {
        SensorInput::reset();
        last_reset_wall_stamp = std::chrono::steady_clock::now();
    }

  protected:
    virtual void onRawDataArrived(const typename Message::ConstPtr& m) = 0;

  private:
    void onRawDataArrivedInternal(const typename Message::ConstPtr& m)
    {
        // clear message queue after reset (didn't find a atomic solution :( )
        if ((std::chrono::steady_clock::now() - last_reset_wall_stamp) > 200ms)
            onRawDataArrived(m);
    }

  protected:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> last_reset_wall_stamp{0ns};
};

} // namespace streaming_clustering

#endif // STREAMING_CLUSTERING_ROS_SENSOR_INPUT_H
