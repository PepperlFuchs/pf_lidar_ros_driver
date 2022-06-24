#include <exception>
#include <limits>
#include <utility>

#include "pf_driver/ros/scan_publisher.h"

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
    if (layer_idx == 0)
    {
      config_mutex_->lock();
    }

    msg.reset(new sensor_msgs::LaserScan());
    msg->header.frame_id.assign(frame_id_);
    msg->header.seq = packet.header.header.scan_number;
    msg->scan_time = 1000.0 / packet.header.scan_frequency;
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
      // in case of single layered device (R2000),
      // h_enabled_layer = 0 and layer_idx is always 0
      if (layer_idx == params_->h_enabled_layer)
      {
        config_mutex_->unlock();
      }
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

void PointcloudPublisher::handle_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx, int layer_inclination,
                                      bool apply_correction)
{
  publish_scan(msg, layer_idx);

  sensor_msgs::PointCloud2 c;
  int channelOptions = laser_geometry::channel_option::Intensity;
  if (apply_correction)
  {
    // since 'apply_correction' calculates the point cloud from laser scan message,
    // 'transformLaserScanToPointCloud' is not be needed anymore
    // just 'projectLaser' is enough just so that it initializes the pointcloud message correctly
    projector_.projectLaser(*msg, c);
    project_laser(c, msg, layer_inclination);
  }
  else
  {
    projector_.transformLaserScanToPointCloud(frame_id_, *msg, c, tfListener_, -1.0, channelOptions);
  }

  if (layer_idx <= layer_prev_)
  {
    if (!cloud_->data.empty())
    {
      cloud_->header.frame_id = frame_id_;
      pcl_publisher_.publish(cloud_);
      cloud_.reset(new sensor_msgs::PointCloud2());
    }
    copy_pointcloud(*cloud_, c);
  }
  else
  {
    add_pointcloud(*cloud_, c);
  }
  layer_prev_ = layer_idx;
}

void PointcloudPublisher::copy_pointcloud(sensor_msgs::PointCloud2& c1, sensor_msgs::PointCloud2 c2)
{
  c1.header.frame_id = c2.header.frame_id;
  c1.height = c2.height;
  c1.width = c2.width;
  c1.is_bigendian = c2.is_bigendian;
  c1.point_step = c2.point_step;
  c1.row_step = c2.row_step;

  c1.fields = std::move(c2.fields);
  c1.data = std::move(c2.data);
}

void PointcloudPublisher::add_pointcloud(sensor_msgs::PointCloud2& c1, sensor_msgs::PointCloud2 c2)
{
  pcl::PCLPointCloud2 p1, p2;
  pcl_conversions::toPCL(c1, p1);
  pcl::PointCloud<pcl::PointXYZI>::Ptr p1_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  // handle when point cloud is empty
  pcl::fromPCLPointCloud2(p1, *p1_cloud);

  pcl_conversions::toPCL(c2, p2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr p2_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(p2, *p2_cloud);

  *p1_cloud += *p2_cloud;
  pcl::toROSMsg(*p1_cloud.get(), c1);
}

void PointcloudPublisher::project_laser(sensor_msgs::PointCloud2& c, sensor_msgs::LaserScanPtr msg,
                                        const int layer_inclination)
{
  pcl::PCLPointCloud2 p;
  pcl_conversions::toPCL(c, p);
  pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  // handle when point cloud is empty
  pcl::fromPCLPointCloud2(p, *p_cloud);

  size_t cl_idx = 0;

  double angle_v_deg = layer_inclination / 10000.0;
  double angle_v = (M_PI / 180.0) * angle_v_deg;

  for (size_t i = 0; i < msg->ranges.size(); i++)
  {
    // num of points in cloud is sometimes less than that in laser scan because of
    // https://github.com/ros-perception/laser_geometry/blob/indigo-devel/src/laser_geometry.cpp#L110
    if (msg->ranges[i] < msg->range_min || msg->ranges[i] >= msg->range_max)
    {
      continue;
    }
    double angle_h = msg->angle_min + msg->angle_increment * (double)i;

    angle_v = correction_params_[layer_inclination][0] * angle_h * angle_h +
              correction_params_[layer_inclination][1] * angle_h + correction_params_[layer_inclination][2];

    p_cloud->points[cl_idx].x = cos(angle_h) * cos(angle_v) * msg->ranges[i];
    p_cloud->points[cl_idx].y = sin(angle_h) * cos(angle_v) * msg->ranges[i];
    p_cloud->points[cl_idx].z = sin(angle_v) * msg->ranges[i];

    cl_idx++;
  }

  pcl::toROSMsg(*p_cloud.get(), c);
}
