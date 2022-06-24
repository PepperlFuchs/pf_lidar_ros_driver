#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
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

    angles_.resize(num_layers);

    for (size_t i = 0; i < angles_param.size(); i++)
    {
      angles_[i] = angles_param[i]["ang"];
      auto coeff_param = angles_param[i]["coeff"];
      std::vector<double> coeffs;
      for (size_t j = 0; j < coeff_param.size(); j++)
      {
        coeffs.push_back(coeff_param[j]);
      }

      correction_params_[angles_[i]] = coeffs;
    }

    for (int i = 0; i < angles_param.size(); i++)
    {
      std::string topic = scan_topic + "_" + std::to_string(i + 1);
      std::string id = frame_id + "_" + std::to_string(i + 1);

      // init frames for each layer
      publish_static_transform(frame_id, id, angles_[i]);

      scan_publishers_.push_back(p_nh.advertise<sensor_msgs::LaserScan>(topic.c_str(), 100));
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

  virtual void resetCurrentScans()
  {
    cloud_.reset(new sensor_msgs::PointCloud2());
    layer_prev_ = -1;
  }

  void publish_static_transform(const std::string parent, const std::string child, int inclination_angle)
  {
    geometry_msgs::TransformStamped transform;

    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = parent.c_str();
    transform.child_frame_id = child.c_str();
    transform.transform.translation.x = 0;
    transform.transform.translation.y = 0;
    transform.transform.translation.z = 0;

    tf2::Quaternion quat;
    double ang = inclination_angle / 10000.0 * M_PI / 180.0;
    quat.setRPY(0, ang, 0);
    transform.transform.rotation.x = quat.x();
    transform.transform.rotation.y = quat.y();
    transform.transform.rotation.z = quat.z();
    transform.transform.rotation.w = quat.w();

    static_broadcaster_.sendTransform(transform);
  }

  void publish_scan(sensor_msgs::LaserScanPtr msg, uint16_t idx)
  {
    ros::Time t = ros::Time::now();
    msg->header.stamp = t;
    msg->header.frame_id = frame_ids_.at(idx);
    scan_publishers_.at(idx).publish(msg);
  }
};
