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
  void to_msg_queue(T& packet, uint16_t layer_idx = 0);
  virtual void handle_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx) = 0;

  virtual void resetCurrentScans()
  {
  }
};

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

  virtual void handle_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx)
  {
    publish_scan(msg, layer_idx);
  }

  void publish_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx)
  {
    ros::Time t = ros::Time::now();
    msg->header.stamp = t;
    scan_publisher_.publish(std::move(msg));
  }
};

class PointcloudPublisher : public PFDataPublisher
{
public:
  PointcloudPublisher(std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params,
                      std::string scan_topic, std::string frame_id, std::shared_ptr<std::mutex> config_mutex,
                      const uint16_t num_layers, std::string part)
    : PFDataPublisher(config, params, config_mutex), layer_prev_(-1)
  {
    ros::NodeHandle p_nh("~/part_" + part);

    for (int i = 0; i < num_layers; i++)
    {
      std::vector<double> correction_param;
      std::string param = "layer_" + std::to_string(i);
      p_nh.getParam(param.c_str(), correction_param);
      correction_params_.push_back(correction_param);
    }
    cloud_.reset(new sensor_msgs::PointCloud2());
    pcl_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(scan_topic, 1);
    header_publisher_ = nh_.advertise<pf_driver::PFR2300Header>("/r2300_header", 1);
    frame_id_.assign(frame_id);
  }

private:
  sensor_msgs::PointCloud2Ptr cloud_;
  // tf::TransformListener tfListener_;
  laser_geometry::LaserProjection projector_;
  ros::Publisher pcl_publisher_;
  int16_t layer_prev_;
  std::vector<std::vector<double>> correction_params_;

  virtual void handle_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx);
  void add_pointcloud(sensor_msgs::PointCloud2& c1, sensor_msgs::PointCloud2 c2);
  void copy_pointcloud(sensor_msgs::PointCloud2& c1, sensor_msgs::PointCloud2 c2);

  void apply_correction(sensor_msgs::PointCloud2& c, sensor_msgs::LaserScanPtr msg, const uint16_t layer_idx);

  virtual void resetCurrentScans()
  {
    cloud_.reset(new sensor_msgs::PointCloud2());
    layer_prev_ = -1;
  }
};