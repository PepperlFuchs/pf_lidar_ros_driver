#pragma once

#include "pf_driver/pf/reader.h"
#include "pf_driver/pf/pf_packet/pf_packet.h"

class PFR2000Packet_A;
class PFR2000Packet_B;
class PFR2000Packet_C;
class PFR2300Packet_C1;

class PFPacketReader : public Reader<PFPacket>, public std::enable_shared_from_this<PFPacketReader>
{
public:
  virtual void read(std::shared_ptr<PFPacket> packet);

  virtual void read(PFR2000Packet_A& packet) = 0;
  virtual void read(PFR2000Packet_B& packet) = 0;
  virtual void read(PFR2000Packet_C& packet) = 0;
  virtual void read(PFR2300Packet_C1& packet) = 0;

  virtual bool start();
  virtual bool stop();
};
