#pragma once

#include <deque>
#include <mutex>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "pf_driver/pf/pf_packet_reader.h"
#include "pf_interfaces/msg/pfr2000_header.hpp"
#include "pf_interfaces/msg/pfr2300_header.hpp"

class PFDataPublisher : public PFPacketReader
{
public:
  PFDataPublisher(std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params);

  virtual void read(PFR2000Packet_A& packet);
  virtual void read(PFR2000Packet_B& packet);
  virtual void read(PFR2000Packet_C& packet);
  virtual void read(PFR2300Packet_C1& packet);

  virtual bool start();

  virtual bool stop();

protected:
  std::string frame_id_;
  std::deque<sensor_msgs::msg::LaserScan::SharedPtr> d_queue_;
  std::mutex q_mutex_;

  std::shared_ptr<ScanConfig> config_;
  std::shared_ptr<ScanParameters> params_;

  bool check_status(uint32_t status_flags);

  template <typename T>
  void to_msg_queue(T& packet, uint16_t layer_idx = 0, int layer_inclination = 0);
  virtual void handle_scan(sensor_msgs::msg::LaserScan::SharedPtr msg, uint16_t layer_idx, int layer_inclination,
                           bool apply_correction = true) = 0;

  virtual void resetCurrentScans()
  {
  }

  virtual void publish_header(pf_interfaces::msg::PFR2000Header& header)
  {
  }

  virtual void publish_header(pf_interfaces::msg::PFR2300Header& header)
  {
  }
};
