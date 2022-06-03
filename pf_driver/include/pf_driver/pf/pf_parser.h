#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "pf_driver/pf/pf_packet.h"

template <typename T>
class Parser
{
public:
  virtual bool parse(uint8_t* buf, size_t buf_len, std::vector<std::unique_ptr<T>>& results, size_t& used, rclcpp::Logger logger) = 0;
};

template <typename T>
class PFParser : public Parser<PFPacket>
{
public:
  virtual bool parse(uint8_t* buf, size_t buf_len, std::vector<std::unique_ptr<PFPacket>>& results, size_t& used, rclcpp::Logger logger) override
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
        RCLCPP_DEBUG(logger, "No magic number found. Invalid packet.");
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
      results.push_back(std::move(packet));
      ++count;

      buf += p_size;
      buf_len -= p_size;
      used = buf - orig_buf;
      packet = std::make_unique<T>();
    }

    if (count == 0)
      RCLCPP_DEBUG(logger, "Received data smaller than header size");

    return count > 0;
  }
};

typedef PFParser<PFR2000Packet_A> PFR2000_A_Parser;
typedef PFParser<PFR2000Packet_B> PFR2000_B_Parser;
typedef PFParser<PFR2000Packet_C> PFR2000_C_Parser;

typedef PFParser<PFR2300Packet_C1> PFR2300_C1_Parser;
