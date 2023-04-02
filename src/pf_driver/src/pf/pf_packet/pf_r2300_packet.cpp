#include "pf_driver/pf/pf_packet/pf_r2300_packet.h"

size_t PFR2300Packet::get_size()
{
  return header_size;
}

std::tuple<uint16_t, uint32_t, uint16_t> PFR2300Packet::read_header(boost::shared_array<uint8_t> buffer)
{
  uint8_t* buf = buffer.get();
  memcpy(&header.header.magic, buf, sizeof(uint16_t));
  buf += sizeof(uint16_t);
  memcpy(&header.header.packet_type, buf, sizeof(uint16_t));
  buf += sizeof(uint16_t);
  memcpy(&header.header.packet_size, buf, sizeof(uint32_t));
  buf += sizeof(uint32_t);
  memcpy(&header.header.header_size, buf, sizeof(uint16_t));
  buf += sizeof(uint16_t);
  memcpy(&header.header.scan_number, buf, sizeof(uint16_t));
  buf += sizeof(uint16_t);
  memcpy(&header.header.packet_number, buf, sizeof(uint16_t));
  buf += sizeof(uint16_t);

  memcpy(&header.layer_index, buf, sizeof(uint16_t));
  buf += sizeof(uint16_t);
  memcpy(&header.layer_inclination, buf, sizeof(uint32_t));
  buf += sizeof(uint32_t);
  memcpy(&header.timestamp_raw, buf, sizeof(uint64_t));
  buf += sizeof(uint64_t);
  memcpy(&header.reserved1, buf, sizeof(uint64_t));
  buf += sizeof(uint64_t);
  memcpy(&header.status_flags, buf, sizeof(uint32_t));
  buf += sizeof(uint32_t);
  memcpy(&header.scan_frequency, buf, sizeof(uint32_t));
  buf += sizeof(uint32_t);
  memcpy(&header.num_points_scan, buf, sizeof(uint16_t));
  buf += sizeof(uint16_t);
  memcpy(&header.num_points_packet, buf, sizeof(uint16_t));
  buf += sizeof(uint16_t);
  memcpy(&header.first_index, buf, sizeof(uint16_t));
  buf += sizeof(uint16_t);
  memcpy(&header.first_angle, buf, sizeof(uint32_t));
  buf += sizeof(uint32_t);
  memcpy(&header.angular_increment, buf, sizeof(uint32_t));
  buf += sizeof(uint32_t);
  memcpy(&header.reserved2, buf, sizeof(uint32_t));
  buf += sizeof(uint32_t);
  memcpy(&header.reserved3, buf, sizeof(uint32_t));
  buf += sizeof(uint32_t);
  memcpy(&header.reserved4, buf, sizeof(uint64_t));
  buf += sizeof(uint64_t);
  memcpy(&header.reserved5, buf, sizeof(uint64_t));
  buf += sizeof(uint64_t);

  return std::tuple<uint16_t, uint32_t, uint16_t>(header.header.header_size, header.header.packet_size,
                                                  header.num_points_packet);
}
