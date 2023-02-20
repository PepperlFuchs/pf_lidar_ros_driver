#pragma once

#include <ros/serialization.h>
#include "pf_driver/PFHeader.h"

class PFPacketReader;

class PFPacket
{
public:
  ros::Time last_acquired_point_stamp;
  pf_driver::PFHeader header;
  std::vector<uint32_t> distance;
  std::vector<uint16_t> amplitude;

  virtual void read_with(PFPacketReader& reader)
  {
  }

  int find_packet_start(uint8_t* buf, size_t buf_len);
  bool parse_buf(uint8_t* buf, size_t buf_len, size_t& remainder, size_t& p_size);

protected:
  virtual size_t get_size() = 0;
  virtual void get_type(char* p_type) = 0;
  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(ros::serialization::IStream& stream) = 0;
  virtual void read_data(uint8_t* buf, size_t num) = 0;
};
