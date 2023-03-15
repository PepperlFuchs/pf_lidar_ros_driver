#include "pf_driver/pf/pf_packet/pf_r2300_packet.h"

size_t PFR2300Packet::get_size()
{
  return 0;  // ros::serialization::serializationLength(header);
}

// TODO
std::tuple<uint16_t, uint32_t, uint16_t> PFR2300Packet::read_header(uint8_t* buf, size_t buf_len, size_t header_len)
{
  return std::tuple<uint16_t, uint32_t, uint16_t>(0, 0, 0);
}