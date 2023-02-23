//#include <exception>
//#include <limits>
//#include <utility>

#include "pf_driver/ros/pf_data_publisher.h"
#include "pf_driver/pf/pf_packet/pf_r2000_packet_a.h"
#include "pf_driver/pf/pf_packet/pf_r2000_packet_b.h"
#include "pf_driver/pf/pf_packet/pf_r2000_packet_c.h"
#include "pf_driver/pf/pf_packet/pf_r2300_packet_c1.h"

PFDataPublisher::PFDataPublisher(std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params)
  : config_(config), params_(params)
{
}

void PFDataPublisher::read(PFR2000Packet_A& packet)
{
  header_publisher_.publish(packet.header);
  to_msg_queue<PFR2000Packet_A>(packet);
}

void PFDataPublisher::read(PFR2000Packet_B& packet)
{
  header_publisher_.publish(packet.header);
  to_msg_queue<PFR2000Packet_B>(packet);
}

void PFDataPublisher::read(PFR2000Packet_C& packet)
{
  header_publisher_.publish(packet.header);
  to_msg_queue<PFR2000Packet_C>(packet);
}

void PFDataPublisher::read(PFR2300Packet_C1& packet)
{
  header_publisher_.publish(packet.header);
  to_msg_queue<PFR2300Packet_C1>(packet, packet.header.layer_index, packet.header.layer_inclination);
}

bool PFDataPublisher::start()
{
  return true;
}

bool PFDataPublisher::stop()
{
  return true;
}

// What are validation checks required here?
// Skipped scans?
// Device errors?
template <typename T>
void PFDataPublisher::to_msg_queue(T& packet, uint16_t layer_idx, int layer_inclination)
{
  if (!check_status(packet.header.status_flags))
    return;

  sensor_msgs::LaserScanPtr msg;
  if (d_queue_.empty())
    d_queue_.emplace_back();
  else if (d_queue_.size() > 5)
    d_queue_.pop_front();
  if (packet.header.header.packet_number == 1)
  {
    const auto scan_time = ros::Duration(1000.0 / packet.header.scan_frequency);
    msg.reset(new sensor_msgs::LaserScan());
    msg->header.frame_id.assign(frame_id_);
    msg->header.seq = packet.header.header.scan_number;
    msg->scan_time = static_cast<float>(scan_time.toSec());
    msg->header.stamp = packet.last_acquired_point_stamp - scan_time;
    msg->angle_increment = packet.header.angular_increment / 10000.0 * (M_PI / 180.0);

    {
      msg->time_increment = (params_->angular_fov * msg->scan_time) / (M_PI * 2.0) / packet.header.num_points_scan;
      msg->angle_min = params_->angle_min;
      msg->angle_max = params_->angle_max;
      if (std::is_same<T, PFR2300Packet_C1>::value)  // Only Packet C1 for R2300
      {
        double config_start_angle = config_->start_angle / 1800000.0 * M_PI;
        if (config_start_angle > params_->angle_min)
        {
          msg->angle_min = config_start_angle;
        }
        if (config_->max_num_points_scan != 0)  // means need to calculate
        {
          double config_angle = (config_->max_num_points_scan - 1) * (params_->scan_freq / 500.0) / 180.0 * M_PI;
          if (msg->angle_min + config_angle < msg->angle_max)
          {
            msg->angle_max = msg->angle_min + config_angle;
          }
        }
      }

      msg->range_min = params_->radial_range_min;
      msg->range_max = params_->radial_range_max;
    }

    msg->ranges.resize(packet.header.num_points_scan);
    if (!packet.amplitude.empty())
      msg->intensities.resize(packet.header.num_points_scan);
    d_queue_.push_back(msg);
  }
  msg = d_queue_.back();
  if (!msg)
    return;

  // errors in scan_number - not in sequence sometimes
  if (msg->header.seq != packet.header.header.scan_number)
    return;
  int idx = packet.header.first_index;

  for (int i = 0; i < packet.header.num_points_packet; i++)
  {
    float data;
    if (packet.distance[i] == 0xFFFFFFFF)
      data = std::numeric_limits<std::uint32_t>::quiet_NaN();
    else
      data = packet.distance[i] / 1000.0;
    msg->ranges[idx + i] = std::move(data);
    if (!packet.amplitude.empty() && packet.amplitude[i] >= 32)
      msg->intensities[idx + i] = packet.amplitude[i];
  }
  if (packet.header.num_points_scan == (idx + packet.header.num_points_packet))
  {
    if (msg)
    {
      handle_scan(msg, layer_idx, layer_inclination, params_->apply_correction);
      d_queue_.pop_back();
    }
  }
}

// check the status bits here with a switch-case
// Currently only for logging purposes only
bool PFDataPublisher::check_status(uint32_t status_flags)
{
  // if(packet.header.header.scan_number > packet.)
  return true;
}
