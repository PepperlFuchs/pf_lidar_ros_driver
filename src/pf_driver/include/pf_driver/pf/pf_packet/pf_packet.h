#pragma once

#include <boost/smart_ptr.hpp>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include "pf_interfaces/msg/pf_header.hpp"

class PFPacketReader;

class PFPacket
{
public:
  rclcpp::Time last_acquired_point_stamp;
  pf_interfaces::msg::PFHeader header;
  std::vector<uint32_t> distance;
  std::vector<uint16_t> amplitude;
  rclcpp::Serialization<pf_interfaces::msg::PFHeader> serialization;

  virtual void read_with(PFPacketReader& reader)
  {
  }

  int find_packet_start(uint8_t* buf, size_t buf_len);
  bool parse_buf(uint8_t* buf, size_t buf_len, size_t& remainder, size_t& p_size);

  virtual ~PFPacket() = default;

protected:
  size_t header_size;
  virtual size_t get_size() = 0;
  virtual void get_type(char* p_type) = 0;
  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(boost::shared_array<uint8_t> buffer) = 0;
  virtual void read_data(uint8_t* buf, size_t num) = 0;
};
