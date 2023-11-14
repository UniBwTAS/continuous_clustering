#ifndef CONTINUOUS_CLUSTERING_OUSTER_INPUT_H
#define CONTINUOUS_CLUSTERING_OUSTER_INPUT_H

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>

#include "ouster_ros/GetMetadata.h"
#include "ouster_ros/PacketMsg.h"

#include "ros_sensor_input.h"

namespace continuous_clustering
{

/*
 * Generic operation to read a channel field from a packet measurement block
 * into a scan
 */
struct parse_field_col
{
    template<typename T>
    void operator()(Eigen::Ref<ouster::img_t<T>> field,
                    ouster::sensor::ChanField f,
                    uint16_t m_id,
                    const ouster::sensor::packet_format& pf,
                    const uint8_t* col_buf)
    {
        // user defined fields that we shouldn't change
        if (f >= ouster::sensor::ChanField::CUSTOM0 && f <= ouster::sensor::ChanField::CUSTOM9)
            return;

        // RAW_HEADERS field is populated separately because it has
        // a different processing scheme and doesn't fit into existing field
        // model (i.e. data packed per column rather than per pixel)
        if (f == ouster::sensor::ChanField::RAW_HEADERS)
            return;

        pf.col_field(col_buf, f, field.col(m_id).data(), field.cols());
    }
};

class OusterInput : public RosSensorInput<ouster_ros::PacketMsg>
{
  public:
    OusterInput(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : RosSensorInput(nh, nh_private)
    {
        ouster_ros::GetMetadata metadata{};
        auto client = nh_.serviceClient<ouster_ros::GetMetadata>("get_metadata");
        client.waitForExistence();
        if (!client.call(metadata))
        {
            auto error_msg = "OusterCloud: Calling get_metadata service failed";
            ROS_ERROR_STREAM(error_msg);
            throw std::runtime_error(error_msg);
        }

        ROS_INFO("OusterCloud: retrieved sensor metadata!");

        info = ouster::sensor::parse_metadata(metadata.response.metadata);
        uint32_t H = info.format.pixels_per_column;
        uint32_t W = info.format.columns_per_frame;

        n_returns =
            info.format.udp_profile_lidar == ouster::sensor::UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL ? 2 :
                                                                                                                    1;

        ROS_INFO_STREAM("Profile has " << n_returns << " return(s)");

        // The ouster_ros drive currently only uses single precision when it
        // produces the point cloud. So it isn't of a benefit to compute point
        //  cloud xyz coordinates using double precision (for the time being).
        auto xyz_lut = ouster::make_xyz_lut(info);
        lut_direction = xyz_lut.direction.cast<float>();
        lut_offset = xyz_lut.offset.cast<float>();
        points = ouster::PointsF(H, lut_offset.cols());

        // reorder flattened array in column major order for faster/easier column-wise access
        ouster::PointsF lut_direction_row_major = lut_direction;
        ouster::PointsF lut_offset_row_major = lut_offset;
        int i_out = 0;
        for (int column_index = 0; column_index < W; column_index++)
        {
            for (int row_index = 0; row_index < H; row_index++)
            {
                int i_in = row_index * static_cast<int>(W) + column_index;
                lut_direction.row(i_out) = lut_direction_row_major.row(i_in);
                lut_offset.row(i_out) = lut_offset_row_major.row(i_in);
                i_out++;
            }
        }

        ls = ouster::LidarScan{1, H, info.format.udp_profile_lidar};
        cloud = ouster_ros::Cloud{1, H};

        pf = std::make_unique<ouster::sensor::packet_format>(info);

        num_lasers = static_cast<int>(H);
        prepareNewFiring();
    }

    void reset() override
    {
        RosSensorInput::reset();
        interrupt_message = true;
    }

  protected:
    void onRawDataArrived(const ouster_ros::PacketMsg::ConstPtr& m) override
    {
        const uint8_t* packet_buf = m->buf.data();

        // const uint16_t f_id = pf->frame_id(packet_buf);

        uint64_t packet_receive_time = ros::Time::now().toNSec();

        // parse measurement blocks
        for (int icol = 0; icol < pf->columns_per_packet; icol++)
        {
            const uint8_t* col_buf = pf->nth_col(icol, packet_buf);
            const uint16_t m_id = pf->col_measurement_id(col_buf);
            const std::chrono::nanoseconds ts(pf->col_timestamp(col_buf));
            // const uint32_t encoder = pf->col_encoder(col_buf);
            const uint32_t status = pf->col_status(col_buf);
            const bool valid = (status & 0x01);

            // drop invalid
            if (!valid)
                continue;

            // write new header values
            ls.timestamp()[0] = ts.count();
            ls.measurement_id()[0] = m_id;
            ls.status()[0] = status;

            ouster::impl::foreach_field(ls, parse_field_col(), 0 /*m_id*/, *pf, col_buf);

            scan_to_cloud_f(points,
                            lut_direction.block(m_id * ls.h, 0, ls.h, 3),
                            lut_offset.block(m_id * ls.h, 0, ls.h, 3),
                            ts,
                            ls,
                            cloud,
                            0 /*return_id*/);

            for (auto& p : cloud)
            {
                if (p.range > 0)
                {
                    current_firing->points[p.ring].x = p.x;
                    current_firing->points[p.ring].y = p.y;
                    current_firing->points[p.ring].z = p.z;

                    // according to https://github.com/ouster-lidar/ouster_example/issues/128#issuecomment-558160200 the
                    // typical intensity range is between 0 - 1000 (can be theoretically higher, up to 6000)
                    current_firing->points[p.ring].intensity =
                        static_cast<uint8_t>(std::min(1.f, p.intensity / 1000.f) * 255);
                }
                else
                {
                    current_firing->points[p.ring].x = nanf("");
                    current_firing->points[p.ring].y = nanf("");
                    current_firing->points[p.ring].z = nanf("");
                    current_firing->points[p.ring].intensity = 0;
                }
                current_firing->points[p.ring].firing_index = firing_index;
                current_firing->points[p.ring].stamp = packet_receive_time;
            }

            if (!interrupt_message)
            {
                publishCurrentFiringAndPrepareNewFiring();
            }
            else
            {
                prepareNewFiring();
                break;
            }
        }

        interrupt_message = false;
    }

  private:
    bool interrupt_message{false};
    ouster::sensor::sensor_info info;
    std::unique_ptr<ouster::sensor::packet_format> pf;
    int n_returns{0};
    ouster::PointsF lut_direction;
    ouster::PointsF lut_offset;
    ouster::PointsF points;
    ouster::LidarScan ls;
    ouster_ros::Cloud cloud;
};

} // namespace continuous_clustering
#endif
