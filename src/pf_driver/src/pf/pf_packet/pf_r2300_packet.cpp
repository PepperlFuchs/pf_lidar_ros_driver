#include "pf_driver/pf/pf_packet/pf_r2300_packet.h"

size_t PFR2300Packet::get_size()
{
  return ros::serialization::serializationLength(header);
}

std::tuple<uint16_t, uint32_t, uint16_t> PFR2300Packet::read_header(ros::serialization::IStream& stream)
{
  ros::serialization::Serializer<pf_driver::PFR2300Header>::read(stream, header);
  return std::tuple<uint16_t, uint32_t, uint16_t>(header.header.header_size, header.header.packet_size,
                                                  header.num_points_packet);
}
