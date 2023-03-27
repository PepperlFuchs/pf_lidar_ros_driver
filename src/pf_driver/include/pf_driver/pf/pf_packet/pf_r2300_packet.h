#pragma once

#include "pf_driver/pf/pf_packet/pf_packet.h"
#include "pf_interfaces/msg/pfr2300_header.hpp"

class PFR2300Packet : public PFPacket
{
public:
  PFR2300Packet()
  {
    header_size = sizeof(pf_interfaces::msg::PFR2300Header);
  }
  virtual size_t get_size();

  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(boost::shared_array<uint8_t> buffer);

  pf_interfaces::msg::PFR2300Header header;
};
