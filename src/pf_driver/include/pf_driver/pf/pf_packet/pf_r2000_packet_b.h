#pragma once

#include "pf_driver/pf/pf_packet/pf_r2000_packet.h"

class PFR2000Packet_B : public PFR2000Packet
{
protected:
#pragma pack(push, pfB, 1)
  struct Data
  {
    uint32_t distance;
    uint16_t amplitude;
  };
#pragma pack(pop, pfB)

  virtual void get_type(char* c);

  virtual void read_data(uint8_t* buf, size_t num);
  virtual void read_with(PFPacketReader& reader);
};
