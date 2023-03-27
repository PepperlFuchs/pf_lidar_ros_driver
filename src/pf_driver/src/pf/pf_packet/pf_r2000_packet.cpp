#include "pf_driver/pf/pf_packet/pf_r2000_packet.h"
#include <iostream>

size_t PFR2000Packet::get_size()
{
  return header_size;
}

std::tuple<uint16_t, uint32_t, uint16_t> PFR2000Packet::read_header(boost::shared_array<uint8_t> buffer)
{
  memcpy((void*)&header, (void*)buffer.get(), header_size);
  return std::tuple<uint16_t, uint32_t, uint16_t>(header.header.header_size, header.header.packet_size,
                                                  header.num_points_packet);
}
