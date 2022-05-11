#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pf_driver/pf/reader.h"
#include "pf_driver/pf/pf_packet.h"
#include "pf_driver/queue/readerwriterqueue.h"

class ScanPublisher : public PFPacketReader
{
public:
  ScanPublisher(std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params,
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
  ros::Publisher scan_publisher_;
  ros::Publisher header_publisher_;
  std::deque<sensor_msgs::LaserScanPtr> d_queue_;
  std::mutex q_mutex_;

  std::shared_ptr<std::mutex> config_mutex_;
  std::shared_ptr<ScanConfig> config_;
  std::shared_ptr<ScanParameters> params_;

  bool check_status(uint32_t status_flags);

  template <typename T>
  void to_msg_queue(T& packet, uint16_t layer_idx = 0);
  virtual void handle_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx) = 0;
  virtual void publish_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx);

  virtual void resetCurrentScans()
  {
  }
};

class ScanPublisherR2000 : public ScanPublisher
{
public:
  ScanPublisherR2000(std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params, std::string scan_topic,
                     std::string frame_id, std::shared_ptr<std::mutex> config_mutex)
    : ScanPublisher(config, params, config_mutex)
  {
    scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(scan_topic, 1);
    header_publisher_ = nh_.advertise<pf_driver::PFR2000Header>("/r2000_header", 1);
    frame_id_ = frame_id;
  }

private:
  virtual void handle_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx)
  {
    publish_scan(msg, layer_idx);
  }
};

class ScanPublisherR2300 : public ScanPublisher
{
public:
  ScanPublisherR2300(std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params, std::string scan_topic,
                     std::string frame_id, std::shared_ptr<std::mutex> config_mutex)
    : ScanPublisher(config, params, config_mutex), tfListener_(nh_), layer_prev_(-1)
  {
    for (int i = 0; i < 4; i++)
    {
      std::string topic = scan_topic + "_" + std::to_string(i + 1);
      std::string id = frame_id + "_" + std::to_string(i + 1);
      scan_publishers_.push_back(nh_.advertise<sensor_msgs::LaserScan>(topic.c_str(), 100));
      frame_ids_.push_back(id);
    }
    cloud_.reset(new sensor_msgs::PointCloud2());
    pcl_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(scan_topic, 1);
    header_publisher_ = nh_.advertise<pf_driver::PFR2300Header>("/r2300_header", 1);
    frame_id_.assign(frame_id);
  }

private:
  sensor_msgs::PointCloud2Ptr cloud_;
  tf::TransformListener tfListener_;
  laser_geometry::LaserProjection projector_;
  ros::Publisher pcl_publisher_;
  std::vector<ros::Publisher> scan_publishers_;
  std::vector<std::string> frame_ids_;
  int16_t layer_prev_;

  virtual void publish_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx);
  virtual void handle_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx);
  void add_pointcloud(sensor_msgs::PointCloud2& c1, sensor_msgs::PointCloud2 c2);
  void copy_pointcloud(sensor_msgs::PointCloud2& c1, sensor_msgs::PointCloud2 c2);

  virtual void resetCurrentScans()
  {
    cloud_.reset(new sensor_msgs::PointCloud2());
    layer_prev_ = -1;
  }
};