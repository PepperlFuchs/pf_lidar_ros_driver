#pragma once

#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>

#include "pf_driver/ros/pf_data_publisher.h"

class PointcloudPublisher : public PFDataPublisher
{
public:
  PointcloudPublisher(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<ScanConfig> config,
                      std::shared_ptr<ScanParameters> params, const std::string& scan_topic,
                      const std::string& frame_id, const uint16_t num_layers, const std::string& part);

private:
  std::shared_ptr<rclcpp::Node> node_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  laser_geometry::LaserProjection projector_;
  rclcpp::Publisher<pf_interfaces::msg::PFR2300Header>::SharedPtr header_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;
  int16_t layer_prev_;
  std::map<int, std::vector<double>> correction_params_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> scan_publishers_;
  std::vector<std::string> frame_ids_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::vector<int> angles_;

  virtual void handle_scan(sensor_msgs::msg::LaserScan::SharedPtr msg, uint16_t layer_idx, int layer_inclination,
                           bool apply_correction);
  void add_pointcloud(sensor_msgs::msg::PointCloud2& c1, sensor_msgs::msg::PointCloud2 c2);
  void copy_pointcloud(sensor_msgs::msg::PointCloud2& c1, sensor_msgs::msg::PointCloud2 c2);

  void project_laser(sensor_msgs::msg::PointCloud2& c, sensor_msgs::msg::LaserScan::SharedPtr msg,
                     const int layer_inclination);

  virtual void resetCurrentScans();

  void publish_static_transform(const std::string& parent, const std::string& child, int inclination_angle);

  virtual void publish_header(pf_interfaces::msg::PFR2300Header& header);
  void publish_scan(sensor_msgs::msg::LaserScan::SharedPtr msg, uint16_t idx);
};
