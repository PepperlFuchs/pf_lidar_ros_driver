#pragma once

#include "pf_driver/pf/pf_packet/pf_packet.h"
#include "pf_interfaces/msg/pfr2000_header.hpp"

class PFR2000Packet : public PFPacket
{
public:
  PFR2000Packet()
  {
    auto serializer = rclcpp::Serialization<pf_interfaces::msg::PFR2000Header>();  
    rclcpp::SerializedMessage serialized_msg;
    serialization.serialize_message(&header, &serialized_msg);
    header_size = serialized_msg.capacity();
  }

  virtual size_t get_size();

  // virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(rclcpp::SerializedMessage& serialized_msg);
  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(uint8_t* buf, size_t buf_len, size_t header_len);
  pf_interfaces::msg::PFR2000Header header;
};
