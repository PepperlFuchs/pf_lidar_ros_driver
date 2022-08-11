#include "pf_driver/pf/pf_packet/pf_r2000_packet_a.h"
#include "pf_driver/pf/pf_packet_reader.h"

void PFR2000Packet_A::read_with(PFPacketReader& reader)
{
  reader.read(*this);
}

void PFR2000Packet_A::get_type(char* c)
{
  c[0] = 0x41;
  c[1] = 0x00;
}

void PFR2000Packet_A::read_data(uint8_t* buf, size_t num)
{
  Data* data = reinterpret_cast<Data*>(buf);
  distance.resize(num);
  for (int i = 0; i < num; i++)
  {
    distance[i] = data[i].distance;
  }
}
