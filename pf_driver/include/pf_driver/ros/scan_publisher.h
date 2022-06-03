#pragma once

#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pf_driver/pf/reader.h"
#include "pf_driver/pf/pf_packet.h"
#include "pf_driver/queue/readerwriterqueue.h"

class ScanPublisher : public PFPacketReader
{
public:
  ScanPublisher(std::shared_ptr<ScanConfig> config,
                std::shared_ptr<ScanParameters> params,
                std::shared_ptr<std::mutex> config_mutex,
                std::shared_ptr<rclcpp::Node> node)
    : config_(config), params_(params), config_mutex_(config_mutex), node_(node)
  {
  }

  virtual bool start()
  {
    return true;
  }

  virtual bool stop()
  {
    return true;
  }

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::string frame_id_;
  std::deque<sensor_msgs::msg::LaserScan::SharedPtr> d_queue_;
  std::mutex q_mutex_;

  std::shared_ptr<std::mutex> config_mutex_;
  std::shared_ptr<ScanConfig> config_;
  std::shared_ptr<ScanParameters> params_;

  bool check_status(uint32_t status_flags);

  template <typename T>
  void to_msg_queue(T& packet, uint16_t layer_idx = 0);
  virtual void handle_scan(sensor_msgs::msg::LaserScan::SharedPtr msg, uint16_t layer_idx) = 0;

  virtual void resetCurrentScans()
  {
  }
};

class ScanPublisherR2000 : public ScanPublisher
{
public:
  ScanPublisherR2000(std::shared_ptr<ScanConfig> config,
                     std::shared_ptr<ScanParameters> params,
                     const std::string &scan_topic,
                     const std::string &frame_id,
                     std::shared_ptr<std::mutex> config_mutex,
                     std::shared_ptr<rclcpp::Node> node)
    : ScanPublisher(config, params, config_mutex, node)
  {
    scan_publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 1);
    header_publisher_ = node_->create_publisher<pf_interfaces::msg::PFR2000Header>("/r2000_header", 1);
    frame_id_ = frame_id;
  }

  virtual void read(PFR2000Packet_A& packet) override;
  virtual void read(PFR2000Packet_B& packet) override;
  virtual void read(PFR2000Packet_C& packet) override;
  virtual void read(PFR2300Packet_C1& packet) override;

private:
  virtual void handle_scan(sensor_msgs::msg::LaserScan::SharedPtr msg, uint16_t layer_idx)
  {
    msg->header.stamp = node_->get_clock()->now();
    scan_publisher_->publish(*msg);
  }

  rclcpp::Publisher<pf_interfaces::msg::PFR2000Header>::SharedPtr header_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
};

class ScanPublisherR2300 : public ScanPublisher
{
public:
  ScanPublisherR2300(std::shared_ptr<ScanConfig> config,
                     std::shared_ptr<ScanParameters> params,
                     std::string scan_topic,
                     std::string frame_id,
                     std::shared_ptr<std::mutex> config_mutex,
                     std::shared_ptr<rclcpp::Node> node)
    : ScanPublisher(config, params, config_mutex, node),
      layer_prev_(-1),
      _tfBuffer(std::make_unique<tf2_ros::Buffer>(node_->get_clock())),
      _tfListener(std::make_unique<tf2_ros::TransformListener>(*_tfBuffer.get()))
  {
    for (int i = 0; i < 4; i++)
    {
      std::string topic = scan_topic + "_" + std::to_string(i + 1);
      std::string id = frame_id + "_" + std::to_string(i + 1);
      scan_publishers_.push_back(node_->create_publisher<sensor_msgs::msg::LaserScan>(topic, 100));
      frame_ids_.push_back(id);
    }
    cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(scan_topic, 1);
    header_publisher_ = node_->create_publisher<pf_interfaces::msg::PFR2300Header>("/r2300_header", 1);
    frame_id_.assign(frame_id);
  }

  virtual void read(PFR2000Packet_A& packet) override;
  virtual void read(PFR2000Packet_B& packet) override;
  virtual void read(PFR2000Packet_C& packet) override;
  virtual void read(PFR2300Packet_C1& packet) override;

private:
  rclcpp::Publisher<pf_interfaces::msg::PFR2300Header>::SharedPtr header_publisher_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_;
  std::unique_ptr<tf2_ros::Buffer> _tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> _tfListener;
  laser_geometry::LaserProjection projector_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> scan_publishers_;
  std::vector<std::string> frame_ids_;
  int16_t layer_prev_;

  virtual void publish_scan(sensor_msgs::msg::LaserScan::SharedPtr msg, uint16_t layer_idx);
  virtual void handle_scan(sensor_msgs::msg::LaserScan::SharedPtr msg, uint16_t layer_idx);
  void add_pointcloud(sensor_msgs::msg::PointCloud2& c1, sensor_msgs::msg::PointCloud2 c2);
  void copy_pointcloud(sensor_msgs::msg::PointCloud2& c1, sensor_msgs::msg::PointCloud2 c2);

  virtual void resetCurrentScans()
  {
    cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    layer_prev_ = -1;
  }
};
