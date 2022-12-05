#pragma once

#include "pf_driver/pf/pf_packet/pf_packet.h"
#include "pf_interfaces/msg/pfr2000_header.hpp"

class PFR2000Packet : public PFPacket
{
public:
  virtual size_t get_size();

  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(rclcpp::SerializedMessage &serialized_msg);

  pf_interfaces::msg::PFR2000Header header;
};
