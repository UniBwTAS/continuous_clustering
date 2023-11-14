#ifndef CONTINUOUS_CLUSTERING_VELODYNE_INPUT_H
#define CONTINUOUS_CLUSTERING_VELODYNE_INPUT_H

#include <ethernet_msgs/Packet.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <continuous_clustering/ros_sensor_input.h>
#include <string>
#include <velodyne_pointcloud/calibration.h>
#include <velodyne_pointcloud/datacontainerbase.h>
#include <velodyne_pointcloud/rawdata.h>

namespace continuous_clustering
{

class VelodyneInput : public velodyne_rawdata::DataContainerBase, public RosSensorInput<ethernet_msgs::Packet>
{
  public:
    VelodyneInput(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
        : DataContainerBase(0, 0, "", "", 0, 0, false, 0, 0), RosSensorInput(nh, nh_private)
    {
        boost::optional<velodyne_pointcloud::Calibration> calibration = parser.setup(nh_private);
        if (calibration)
        {
            num_lasers = calibration->num_lasers;
            prepareNewFiring();
        }
        else
            throw std::runtime_error("Could not load calibration file!");
        parser.setParameters(0, 300, 0, 2 * M_PI);
    }

    void reset() override
    {
        RosSensorInput::reset();
        interrupt_message = true;
    }

    void newLine() override
    {
        if (!interrupt_message)
            publishCurrentFiringAndPrepareNewFiring();
        else
            prepareNewFiring();
    }

    void addPoint(float x,
                  float y,
                  float z,
                  const uint16_t ring,
                  const uint16_t azimuth,
                  const float distance,
                  const float intensity,
                  const float time) override
    {
        int row_index = num_lasers - ring - 1;

        auto offset_in_ns = static_cast<uint64_t>(time / 1e9);
        current_firing->points[row_index].stamp = cur_packet_stamp.toNSec() + offset_in_ns;
        current_firing->points[row_index].firing_index = firing_index;

        if (distance > 0)
        {
            current_firing->points[row_index].x = x;
            current_firing->points[row_index].y = y;
            current_firing->points[row_index].z = z;
            current_firing->points[row_index].intensity = static_cast<uint8_t>(intensity);
        }
        else
        {
            current_firing->points[row_index].x = nanf("");
            current_firing->points[row_index].y = nanf("");
            current_firing->points[row_index].z = nanf("");
            current_firing->points[row_index].intensity = 0;
        }
    }

  protected:
    void onRawDataArrived(const ethernet_msgs::Packet::ConstPtr& m) override
    {
        cur_packet_stamp = m->header.stamp;

        // create dummy velodyne packet in order to use unpack function
        velodyne_msgs::VelodynePacket tmp_packet;
        std::memcpy(&tmp_packet.data, &m->payload[0], m->payload.size());
        tmp_packet.stamp = m->header.stamp;

        // only the time diff since packet stamp is calculated and has to added to packet stamp (passed above) later
        interrupt_message = false;
        parser.unpack(tmp_packet, *this, m->header.stamp, 0);
    }

  private:
    velodyne_rawdata::RawData parser;
    ros::Time cur_packet_stamp{0, 0};
    bool interrupt_message{false};
};

} // namespace continuous_clustering
#endif
