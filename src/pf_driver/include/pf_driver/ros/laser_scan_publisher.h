#pragma once

#include "pf_driver/ros/pf_data_publisher.h"

class LaserscanPublisher : public PFDataPublisher
{
public:
  LaserscanPublisher(std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params,
                     const std::string& scan_topic, const std::string& frame_id);

private:
  ros::Publisher scan_publisher_;

  virtual void handle_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx, int layer_inclination,
                           bool apply_correction);

  void publish_scan(sensor_msgs::LaserScanPtr msg);
};
