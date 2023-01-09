#pragma once

#include "pf_driver/pf/pf_packet/pf_packet.h"
#include "pf_driver/PFR2300Header.h"

class PFR2300Packet : public PFPacket
{
public:
  virtual size_t get_size();

  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(ros::serialization::IStream& stream);

  pf_driver::PFR2300Header header;
};
