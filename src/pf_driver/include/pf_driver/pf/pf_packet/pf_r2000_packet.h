#pragma once

#include "pf_driver/pf/pf_packet/pf_packet.h"
#include "pf_driver/PFR2000Header.h"

class PFR2000Packet : public PFPacket
{
public:
  virtual size_t get_size();

  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(ros::serialization::IStream& stream);

  pf_driver::PFR2000Header header;
};
