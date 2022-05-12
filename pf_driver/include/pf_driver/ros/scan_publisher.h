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
  void to_msg_queue(T& packet, uint16_t layer_idx = 0, int layer_inclination = 0);
  virtual void handle_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx, int layer_inclination,
                           bool apply_correction = true) = 0;

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

class PointcloudPublisher : public PFDataPublisher
{
public:
  PointcloudPublisher(std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params,
                      std::string scan_topic, std::string frame_id, std::shared_ptr<std::mutex> config_mutex,
                      const uint16_t num_layers, std::string part)
    : PFDataPublisher(config, params, config_mutex), layer_prev_(-1)
  {
    ros::NodeHandle p_nh("~/");

    XmlRpc::XmlRpcValue angles_param;
    p_nh.getParam("correction_params", angles_param);

    for (size_t i = 0; i < angles_param.size(); i++)
    {
      int ang = angles_param[i]["ang"];
      auto coeff_param = angles_param[i]["coeff"];
      std::vector<double> coeffs;
      for (size_t j = 0; j < coeff_param.size(); j++)
      {
        coeffs.push_back(coeff_param[j]);
      }

      correction_params_[ang] = coeffs;
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
  std::map<int, std::vector<double>> correction_params_;

  virtual void handle_scan(sensor_msgs::LaserScanPtr msg, uint16_t layer_idx, int layer_inclination,
                           bool apply_correction);
  void add_pointcloud(sensor_msgs::PointCloud2& c1, sensor_msgs::PointCloud2 c2);
  void copy_pointcloud(sensor_msgs::PointCloud2& c1, sensor_msgs::PointCloud2 c2);

  void project_laser(sensor_msgs::PointCloud2& c, sensor_msgs::LaserScanPtr msg, const int layer_inclination,
                     bool apply_correction);

  virtual void resetCurrentScans()
  {
    cloud_.reset(new sensor_msgs::PointCloud2());
    layer_prev_ = -1;
  }
};