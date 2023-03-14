#pragma once

#include <boost/smart_ptr.hpp>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include "pf_interfaces/msg/pf_header.hpp"

class PFPacketReader;
#pragma pack(1)
struct PacketHeaderData
{
  //! Magic bytes, must be  5C A2 (hex)
  std::uint16_t magic;

  //! Packet type, must be 43 00 (hex)
  std::uint16_t packet_type;

  //! Overall packet size (header+payload), 1404 bytes with maximum payload
  std::uint32_t packet_size;

  //! Header size, defaults to 60 bytes
  std::uint16_t header_size;

  //! Sequence for scan (incremented for every scan, starting with 0, overflows)
  std::uint16_t scan_number;

  //! Sequence number for packet (counting packets of a particular scan, starting with 1)
  std::uint16_t packet_number;

  //! Raw timestamp of internal clock in NTP time format
  std::uint64_t timestamp_raw;

  //! With an external NTP server synced Timestamp  (currenty not available and and set to zero)
  std::uint64_t timestamp_sync;

  //! Status flags
  std::uint32_t status_flags;

  //! Frequency of scan-head rotation in mHz (Milli-Hertz)
  std::uint32_t scan_frequency;

  //! Total number of scan points (samples) within complete scan
  std::uint16_t num_points_scan;

  //! Total number of scan points within this packet
  std::uint16_t num_points_packet;

  //! Index of first scan point within this packet
  std::uint16_t first_index;

  //! Absolute angle of first scan point within this packet in 1/10000°
  std::int32_t first_angle;

  //! Delta between two succeding scan points 1/10000°
  std::int32_t angular_increment;

  //! Output status
  std::uint32_t output_status;

  //! Field status
  std::uint32_t field_status;

  //! Possible padding to align header size to 32bit boundary
  // std::uint8 padding[0];
};
#pragma pack()
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

protected:
  size_t header_size;
  virtual size_t get_size() = 0;
  virtual void get_type(char* p_type) = 0;
  // virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(rclcpp::SerializedMessage& serialized_msg) = 0;
  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(uint8_t* buf, size_t buf_len, size_t header_len) = 0;
  virtual void read_data(uint8_t* buf, size_t num) = 0;
};
