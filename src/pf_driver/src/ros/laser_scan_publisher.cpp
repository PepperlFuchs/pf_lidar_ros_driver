#include "pf_driver/ros/laser_scan_publisher.h"

#include "pf_interfaces/msg/pfr2000_header.hpp"

LaserscanPublisher::LaserscanPublisher(std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params,
                                       const std::string& scan_topic, const std::string& frame_id)
  : PFDataPublisher(config, params)
{
  scan_publisher_ = nh_.advertise<sensor_msgs::msg::LaserScan>(scan_topic, 1);
  header_publisher_ = nh_.advertise<pf_interfaces::msg::PFR2000Header>("/r2000_header", 1);
  frame_id_ = frame_id;
}

void LaserscanPublisher::handle_scan(sensor_msgs::msg::LaserScan::SharedPtr msg, uint16_t layer_idx, int layer_inclination,
                                     bool apply_correction)
{
  publish_scan(msg);
}

void LaserscanPublisher::publish_scan(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  scan_publisher_.publish(msg);
}
