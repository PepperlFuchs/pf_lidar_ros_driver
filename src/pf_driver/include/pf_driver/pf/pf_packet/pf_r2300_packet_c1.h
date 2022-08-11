#pragma once

#include "pf_driver/pf/pf_packet/pf_r2300_packet.h"

class PFR2300Packet_C1 : public PFR2300Packet
{
protected:
#pragma pack(push, pfC1, 1)
  struct Data
  {
    uint32_t dist_amp;
  };
#pragma pack(pop, pfC1)

  virtual void get_type(char* c);

  virtual void read_data(uint8_t* buf, size_t num);
  virtual void read_with(PFPacketReader& reader);
};
