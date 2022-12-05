#include "pf_driver/pf/pf_packet/pf_r2000_packet.h"

size_t PFR2000Packet::get_size()
{
  return 0;  // ros::serialization::serializationLength(header);
}

std::tuple<uint16_t, uint32_t, uint16_t> PFR2000Packet::read_header(rclcpp::SerializedMessage& serialized_msg)
{
  serialization.deserialize_message(&serialized_msg, &header);
  return std::tuple<uint16_t, uint32_t, uint16_t>(header.header.header_size, header.header.packet_size,
                                                  header.num_points_packet);
}
