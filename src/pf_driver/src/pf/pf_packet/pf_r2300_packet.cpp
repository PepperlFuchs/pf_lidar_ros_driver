#include "pf_driver/pf/pf_packet/pf_r2300_packet.h"

size_t PFR2300Packet::get_size()
{
  return 0;  // ros::serialization::serializationLength(header);
}
/*
std::tuple<uint16_t, uint32_t, uint16_t> PFR2300Packet::read_header(rclcpp::SerializedMessage& serialized_msg)
{
  serialization.deserialize_message(&serialized_msg, &header);
  return std::tuple<uint16_t, uint32_t, uint16_t>(header.header.header_size, header.header.packet_size,
                                                  header.num_points_packet);
}
*/
// TODO
std::tuple<uint16_t, uint32_t, uint16_t> PFR2300Packet::read_header(uint8_t* buf, size_t buf_len, size_t header_len)
{
  return std::tuple<uint16_t, uint32_t, uint16_t>(0, 0,
                                                  0);
}