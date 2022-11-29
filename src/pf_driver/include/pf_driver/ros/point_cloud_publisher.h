#pragma once

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "pf_driver/ros/pf_data_publisher.h"

class PointcloudPublisher : public PFDataPublisher
{
public:
  PointcloudPublisher(std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params,
                      const std::string& scan_topic, const std::string& frame_id, const uint16_t num_layers,
                      const std::string& part);

private:
  sensor_msgs::PointCloud2Ptr cloud_;
  tf::TransformListener tfListener_;
  laser_geometry::LaserProjection projector_;
  ros::Publisher pcl_publisher_;
  int16_t layer_prev_;
  std::map<int, std::vector<double>> correction_params_;
  std::vector<ros::Publisher> scan_publishers_;
  std::vector<std::string> frame_ids_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  std::vector<int> angles_;

  virtual void handle_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx, int layer_inclination,
                           bool apply_correction);
  void add_pointcloud(sensor_msgs::PointCloud2& c1, sensor_msgs::PointCloud2 c2);
  void copy_pointcloud(sensor_msgs::PointCloud2& c1, sensor_msgs::PointCloud2 c2);

  void project_laser(sensor_msgs::PointCloud2& c, sensor_msgs::LaserScanPtr msg, const int layer_inclination);

  virtual void resetCurrentScans();

  void publish_static_transform(const std::string& parent, const std::string& child, int inclination_angle);

  void publish_scan(sensor_msgs::LaserScanPtr msg, uint16_t idx);
};
