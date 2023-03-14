#pragma once

#include "pf_driver/pf/pf_packet/pf_packet.h"
#include "pf_interfaces/msg/pfr2300_header.hpp"

class PFR2300Packet : public PFPacket
{
public:
  virtual size_t get_size();

  // virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(rclcpp::SerializedMessage& serialized_msg);
  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(uint8_t* buf, size_t buf_len, size_t header_len);
  pf_interfaces::msg::PFR2300Header header;
};
