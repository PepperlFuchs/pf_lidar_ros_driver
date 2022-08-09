#pragma once

#include "pf_driver/pf/pf_packet/pf_r2000_packet.h"

class PFR2000Packet_A : public PFR2000Packet
{
protected:
#pragma pack(push, pfA, 1)
  struct Data
  {
    uint32_t distance;
  };
#pragma pack(pop, pfA)

  virtual void get_type(char* c)
  {
    c[0] = 0x41;
    c[1] = 0x00;
  }

  virtual void read_data(uint8_t* buf, size_t num);
  virtual void read_with(PFPacketReader& reader);
};
