#include "pf_driver/pf/pf_packet/pf_r2000_packet.h"

size_t PFR2000Packet::get_size()
{
  return ros::serialization::serializationLength(header);
}

std::tuple<uint16_t, uint32_t, uint16_t> PFR2000Packet::read_header(ros::serialization::IStream& stream)
{
  ros::serialization::Serializer<pf_driver::PFR2000Header>::read(stream, header);
  return std::tuple<uint16_t, uint32_t, uint16_t>(header.header.header_size, header.header.packet_size,
                                                  header.num_points_packet);
}
