#pragma once

//#include <ros/ros.h>
//#include <sensor_msgs/LaserScan.h>
//#include <laser_geometry/laser_geometry.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <tf2_ros/static_transform_broadcaster.h>
//#include <tf/transform_listener.h>
//#include <pcl_conversions/pcl_conversions.h>

//#include "pf_driver/PFR2000Header.h"
//#include "pf_driver/PFR2300Header.h"
#include "pf_driver/ros/pf_data_publisher.h"
//#include "pf_driver/queue/readerwriterqueue.h"

class LaserscanPublisher : public PFDataPublisher
{
public:
  LaserscanPublisher(std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params, std::string scan_topic,
                     std::string frame_id, std::shared_ptr<std::mutex> config_mutex)
    : PFDataPublisher(config, params, config_mutex)
  {
    scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(scan_topic, 1);
    header_publisher_ = nh_.advertise<pf_driver::PFR2000Header>("/r2000_header", 1);
    frame_id_ = frame_id;
  }

private:
  ros::Publisher scan_publisher_;

  virtual void handle_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx, int layer_inclination,
                           bool apply_correction)
  {
    publish_scan(msg);
  }

  void publish_scan(sensor_msgs::LaserScanPtr msg)
  {
    ros::Time t = ros::Time::now();
    msg->header.stamp = t;
    scan_publisher_.publish(std::move(msg));
  }
};
