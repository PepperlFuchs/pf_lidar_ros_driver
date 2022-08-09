#pragma once

#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
//#include <laser_geometry/laser_geometry.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <tf2_ros/static_transform_broadcaster.h>
//#include <tf/transform_listener.h>
//#include <pcl_conversions/pcl_conversions.h>

//#include "pf_driver/PFR2000Header.h"
//#include "pf_driver/PFR2300Header.h"
#include "pf_driver/pf/pf_packet_reader.h"
//#include "pf_driver/queue/readerwriterqueue.h"

class PFDataPublisher : public PFPacketReader
{
public:
  PFDataPublisher(std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params,
                  std::shared_ptr<std::mutex> config_mutex)
    : config_(config), params_(params), config_mutex_(config_mutex)
  {
  }

  virtual void read(PFR2000Packet_A& packet);
  virtual void read(PFR2000Packet_B& packet);
  virtual void read(PFR2000Packet_C& packet);
  virtual void read(PFR2300Packet_C1& packet);

  virtual bool start()
  {
    return true;
  }

  virtual bool stop()
  {
    return true;
  }

protected:
  ros::NodeHandle nh_;
  std::string frame_id_;
  ros::Publisher header_publisher_;
  std::deque<sensor_msgs::LaserScanPtr> d_queue_;
  std::mutex q_mutex_;

  std::shared_ptr<std::mutex> config_mutex_;
  std::shared_ptr<ScanConfig> config_;
  std::shared_ptr<ScanParameters> params_;

  bool check_status(uint32_t status_flags);

  template <typename T>
  void to_msg_queue(T& packet, uint16_t layer_idx = 0, int layer_inclination = 0);
  virtual void handle_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx, int layer_inclination,
                           bool apply_correction = true) = 0;

  virtual void resetCurrentScans()
  {
  }
};
