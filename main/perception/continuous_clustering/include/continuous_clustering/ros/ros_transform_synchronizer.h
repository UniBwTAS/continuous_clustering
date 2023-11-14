#ifndef CONTINUOUS_CLUSTERING_ROS_TRANSFORM_SYNCHRONIZER_H
#define CONTINUOUS_CLUSTERING_ROS_TRANSFORM_SYNCHRONIZER_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <utility>

template<typename Message>
class RosTransformSynchronizer
{
  public:
    explicit RosTransformSynchronizer(const std::shared_ptr<tf2_ros::Buffer>& buffer)
    {
        tf_buffer = buffer;
    }

    explicit RosTransformSynchronizer()
    {
        tf_buffer = std::make_shared<tf2_ros::Buffer>();
        tf_listener =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer, true, ros::TransportHints().tcpNoDelay());
    }

    void reset(const std::string& target_frame, const std::string& source_frame)
    {
        target_frame_ = target_frame;
        source_frame_ = source_frame;
        frames_are_different_ = target_frame_ != source_frame_;
        message_queue.clear();
    }

    void
    setCallback(std::function<void(const typename Message::ConstPtr&, const geometry_msgs::TransformStamped&)> callback)
    {
        callback_ = callback;
    }

    template<class T>
    void setCallback(void (T::*callback)(const typename Message::ConstPtr&, const geometry_msgs::TransformStamped&),
                     T* obj)
    {
        setCallback(std::bind(callback, obj, std::placeholders::_1, std::placeholders::_2));
    }

    void addMessage(const ros::Time& stamp, const typename Message::ConstPtr& msg)
    {
        // save current message in queue
        message_queue.push_back({stamp, msg});

        // get latest available transform odom from sensor frame
        ros::Time latest_available_tf(0, 0);
        bool wait_for_transform_required = wait_for_transform_ && frames_are_different_;
        if (wait_for_transform_required)
        {
            try
            {
                geometry_msgs::TransformStamped tf =
                    tf_buffer->lookupTransform(target_frame_, source_frame_, ros::Time());
                latest_available_tf = tf.header.stamp;
            }
            catch (tf2::TransformException& ex)
            {
                ROS_WARN_STREAM_THROTTLE(1, "Transform not available yet.");
            }
        }
        else
        {
            // pretend that transform is available
            latest_available_tf.sec = std::numeric_limits<int32_t>::max();
        }

        // now check which of the existing messages can be processed because their TF is available (we assume the queue
        // is ordered by timestamp)
        while (!message_queue.empty() && message_queue.front().first <= latest_available_tf)
        {
            // transform again but this time with the specific time stamp
            try
            {
                ros::Time s = wait_for_transform_required ? message_queue.front().first : ros::Time();
                geometry_msgs::TransformStamped tf = tf_buffer->lookupTransform(target_frame_, source_frame_, s);
                if (callback_)
                    callback_(message_queue.front().second, tf);
            }
            catch (tf2::TransformException& ex)
            {
                // even though a tf after our message was available, it can still fail for the first messages,
                // because the buffer has not tf before our first message to interpolate. Simply discard these
                // messages as we can not transform them.
            }
            message_queue.pop_front();
        }
    }

    std::shared_ptr<tf2_ros::Buffer> getTfBuffer()
    {
        return tf_buffer;
    }

    void waitForTransform(bool wait = true)
    {
        wait_for_transform_ = wait;
    }

  private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::deque<std::pair<ros::Time, const typename Message::ConstPtr>> message_queue;
    std::string source_frame_;
    std::string target_frame_;
    std::function<void(const typename Message::ConstPtr&, const geometry_msgs::TransformStamped&)> callback_;
    bool wait_for_transform_{true};
    bool frames_are_different_{false};
};

#endif // CONTINUOUS_CLUSTERING_ROS_TRANSFORM_SYNCHRONIZER_H
