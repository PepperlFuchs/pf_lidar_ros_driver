#include "pf_driver/ros/laser_scan_publisher.h"

LaserscanPublisher::LaserscanPublisher(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<ScanConfig> config,
                                       std::shared_ptr<ScanParameters> params, const std::string& scan_topic,
                                       const std::string& frame_id)
  : PFDataPublisher(config, params), node_(node)
{
  scan_publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS());
  header_publisher_ = node_->create_publisher<pf_interfaces::msg::PFR2000Header>("/r2000_header", 1);
  frame_id_ = frame_id;
}

void LaserscanPublisher::handle_scan(sensor_msgs::msg::LaserScan::SharedPtr msg, uint16_t layer_idx,
                                     int layer_inclination, bool apply_correction)
{
  publish_scan(msg);
}

void LaserscanPublisher::publish_header(pf_interfaces::msg::PFR2000Header& header)
{
  header_publisher_->publish(header);
}

void LaserscanPublisher::publish_scan(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  scan_publisher_->publish(*msg);
}
