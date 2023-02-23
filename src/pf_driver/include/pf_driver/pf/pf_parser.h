#pragma once

#include <ros/ros.h>
#include "pf_driver/pf/parser.h"
#include "pf_driver/pf/pf_packet/pf_r2000_packet_a.h"
#include "pf_driver/pf/pf_packet/pf_r2000_packet_b.h"
#include "pf_driver/pf/pf_packet/pf_r2000_packet_c.h"
#include "pf_driver/pf/pf_packet/pf_r2300_packet_c1.h"

template <typename T>
class PFParser : public Parser<PFPacket>
{
public:
  virtual bool parse(uint8_t* buf, size_t buf_len, std::vector<std::unique_ptr<PFPacket>>& results, size_t& used)
  {
    std::unique_ptr<T> packet = std::make_unique<T>();
    uint32_t serial_size = packet->get_size();
    uint8_t* orig_buf = buf;
    int count = 0;
    used = 0;

    while (buf_len >= serial_size)
    {
      int start = packet->find_packet_start(buf, buf_len);
      if (start == -1)
      {
        ROS_DEBUG("No magic number found. Invalid packet.");
        break;
      }
      buf += start;
      buf_len -= start;
      if (buf_len < serial_size)
        break;
      size_t remainder = 0;
      size_t p_size = 0;
      if (!packet->parse_buf(buf, buf_len, remainder, p_size))
        break;
      packet->last_acquired_point_stamp = ros::Time::now();
      results.push_back(std::move(packet));
      ++count;

      buf += p_size;
      buf_len -= p_size;
      used = buf - orig_buf;
      packet = std::make_unique<T>();
    }

    if (count == 0)
      ROS_DEBUG("Received data smaller than header size");

    return count > 0;
  }
};

using PFR2000_A_Parser = PFParser<PFR2000Packet_A>;
using PFR2000_B_Parser = PFParser<PFR2000Packet_B>;
using PFR2000_C_Parser = PFParser<PFR2000Packet_C>;

using PFR2300_C1_Parser = PFParser<PFR2300Packet_C1>;
