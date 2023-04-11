#pragma once

#include <rclcpp/rclcpp.hpp>

#include "pf_driver/ros/pf_data_publisher.h"

class LaserscanPublisher : public PFDataPublisher
{
public:
  LaserscanPublisher(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<ScanConfig> config,
                     std::shared_ptr<ScanParameters> params, const std::string& scan_topic,
                     const std::string& frame_id);

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<pf_interfaces::msg::PFR2000Header>::SharedPtr header_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;

  virtual void handle_scan(sensor_msgs::msg::LaserScan::SharedPtr msg, uint16_t layer_idx, int layer_inclination,
                           bool apply_correction);

  virtual void publish_header(pf_interfaces::msg::PFR2000Header& header);

  void publish_scan(sensor_msgs::msg::LaserScan::SharedPtr msg);
};
