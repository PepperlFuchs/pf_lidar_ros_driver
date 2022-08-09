#pragma once

#include "pf_driver/pf/pf_packet/pf_packet.h"
#include "pf_driver/PFR2300Header.h"

class PFR2300Packet : public PFPacket
{
public:
  virtual size_t get_size()
  {
    return ros::serialization::serializationLength(header);
  }

  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(ros::serialization::IStream& stream)
  {
    ros::serialization::Serializer<pf_driver::PFR2300Header>::read(stream, header);
    return std::tuple<uint16_t, uint32_t, uint16_t>(header.header.header_size, header.header.packet_size,
                                                    header.num_points_packet);
  }

  pf_driver::PFR2300Header header;
};
