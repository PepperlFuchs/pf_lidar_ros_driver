#include "pf_driver/pf/pf_packet/pf_r2000_packet.h"
#include <iostream>

size_t PFR2000Packet::get_size()
{
  return header_size;
}
/*
std::tuple<uint16_t, uint32_t, uint16_t> PFR2000Packet::read_header(rclcpp::SerializedMessage& serialized_msg)
{
  serialization.deserialize_message(&serialized_msg, &header);
  return std::tuple<uint16_t, uint32_t, uint16_t>(header.header.header_size, header.header.packet_size,
                                                  header.num_points_packet);
}
*/
std::tuple<uint16_t, uint32_t, uint16_t> PFR2000Packet::read_header(uint8_t* buf, size_t buf_len, size_t header_len)
{
  char buffer[65536];
  PacketHeaderData * p = reinterpret_cast<PacketHeaderData *>(buffer);
  char * pp = reinterpret_cast<char *>(p);
  if (buf_len < header_len) {
    throw std::exception();
  }

  memcpy(pp, buf, header_len);
  header.header.magic = p->magic;
  header.header.packet_type = p->packet_type;
  header.header.packet_size = p->packet_size;
  header.header.header_size = p->header_size;
  header.header.scan_number = p->scan_number;
  header.header.packet_number = p->packet_number;
  
  header.timestamp_raw = p->timestamp_raw;
  header.timestamp_sync = p->timestamp_sync;
  header.status_flags = p->status_flags;
  header.scan_frequency = p->scan_frequency;
  header.num_points_scan = p->num_points_scan;
  header.num_points_packet = p->num_points_packet;
  header.first_index = p->first_index;
  header.first_angle = p->first_angle;
  header.angular_increment = p->angular_increment;
  return std::tuple<uint16_t, uint32_t, uint16_t>(p->header_size, p->packet_size,
                                                  p->num_points_packet);
}