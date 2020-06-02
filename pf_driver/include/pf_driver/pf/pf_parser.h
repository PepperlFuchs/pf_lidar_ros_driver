#pragma once

#include <ros/ros.h>
#include "pf_driver/pf/pf_packet.h"

template <typename T>
class Parser
{
public:
  virtual bool parse(uint8_t* buf, size_t buf_len, std::vector<std::unique_ptr<T>>& results) = 0;
};

template <typename T>
class PFParser : public Parser<PFPacket>
{
public:
  virtual bool parse(uint8_t* buf, size_t buf_len, std::vector<std::unique_ptr<PFPacket>>& results)
  {
    std::unique_ptr<T> packet = std::make_unique<T>();

    uint32_t serial_size = packet->get_size();
    if(buf_len < serial_size)
    {
      ROS_ERROR("Received data smaller than header size");
      return false;
    }

    int start = packet->find_packet_start(buf, buf_len);
    if(start == -1)
    {
      ROS_DEBUG("No magic number found. Invalid packet.");
      return false;
    }
    size_t remainder = 0;
    size_t p_size = 0;
    if(!packet->parse_buf(buf + start, buf_len, remainder, p_size))
      return false;

    results.push_back(std::move(packet));
    if(remainder > serial_size)
    {
      return parse(buf + p_size, remainder, results);
    }
    return true;
  }
};

typedef PFParser<PFR2000Packet_A> PFR2000_A_Parser;
typedef PFParser<PFR2000Packet_B> PFR2000_B_Parser;
typedef PFParser<PFR2000Packet_C> PFR2000_C_Parser;

typedef PFParser<PFR2300Packet_C1> PFR2300_C1_Parser;