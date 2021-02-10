#pragma once

#include "pf_driver/pf/pipeline.h"
#include "pf_driver/pf/pf_packet.h"

class PFPacketReader : public Reader<PFPacket>, public std::enable_shared_from_this<PFPacketReader>
{
public:
  virtual void read(std::shared_ptr<PFPacket> packet)
  {
    packet->read_with(*shared_from_this());
  }

  virtual void read(PFR2000Packet_A& packet) = 0;
  virtual void read(PFR2000Packet_B& packet) = 0;
  virtual void read(PFR2000Packet_C& packet) = 0;
  virtual void read(PFR2300Packet_C1& packet) = 0;

  virtual bool start()
  {
    return false;
  }
  virtual bool stop()
  {
    return false;
  }
};
