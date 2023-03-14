#include "pf_driver/pf/pf_packet_reader.h"

void PFPacketReader::read(std::shared_ptr<PFPacket> packet)
{
  packet->read_with(*this);
}

bool PFPacketReader::start()
{
  return false;
}

bool PFPacketReader::stop()
{
  return false;
}
