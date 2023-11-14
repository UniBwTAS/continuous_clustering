#ifndef STREAMING_CLUSTERING_ROS_SENSOR_INPUT_H
#define STREAMING_CLUSTERING_ROS_SENSOR_INPUT_H

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <streaming_clustering/sensor_input.h>
#include "thread_pool.h"

namespace streaming_clustering
{

template<typename Message>
struct MessageJob
{
    const typename Message::ConstPtr msg;
};

template<typename Message>
class RosSensorInput : public SensorInput
{
  public:
    RosSensorInput(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh)
    {
        message_thread_pool_.init([this](MessageJob<Message>&& job) { onRawDataArrived(job.msg); });
    }

    void subscribe() override
    {
        sub = nh_.subscribe("raw_data",
                           100000,
                           &RosSensorInput::onRawDataArrivedInternal,
                           this,
                           ros::TransportHints().tcpNoDelay(true));
    }

    size_t dataCount() override
    {
        return message_thread_pool_.getNumberOfUnprocessedJobs();
    }

    void reset() override
    {
        SensorInput::reset();
        message_thread_pool_.clearJobs();
    }

  protected:
    virtual void onRawDataArrived(const typename Message::ConstPtr& m) = 0;

  private:
    void onRawDataArrivedInternal(const typename Message::ConstPtr& msg)
    {
        message_thread_pool_.enqueue({msg});
    }

  protected:
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    ThreadPool<MessageJob<Message>> message_thread_pool_{"M"};
};

} // namespace streaming_clustering

#endif // STREAMING_CLUSTERING_ROS_SENSOR_INPUT_H
