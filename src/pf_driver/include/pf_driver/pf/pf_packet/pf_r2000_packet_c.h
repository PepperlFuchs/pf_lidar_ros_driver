#pragma once

#include "pf_driver/pf/pf_packet/pf_r2000_packet.h"

class PFR2000Packet_C : public PFR2000Packet
{
protected:
#pragma pack(push, pfC, 1)
  struct Data
  {
    uint32_t dist_amp;
  };
#pragma pack(pop, pfC)

  virtual void get_type(char* c);

  virtual void read_data(uint8_t* buf, size_t num);
  virtual void read_with(PFPacketReader& reader);
};
