#include <algorithm>

#include "pf_driver/pf/pf_packet.h"
#include "pf_driver/pf/reader.h"

bool PFPacket::parse_buf(uint8_t* buf, size_t buf_len, size_t& remainder, size_t& packet_size)
{
  const size_t SIZE = get_size();
  boost::shared_array<uint8_t> buffer(new uint8_t[SIZE]);
  std::copy(buf, buf + SIZE, buffer.get());
  ros::serialization::IStream stream(buffer.get(), SIZE);
  uint16_t h_size;
  uint32_t p_size;
  uint16_t num;
  std::tie(h_size, p_size, num) = read_header(stream);

  auto data_size = p_size - h_size;
  if (buf_len < p_size)
    return false;

  read_data(&buf[h_size], num);
  remainder = buf_len - p_size;
  packet_size = p_size;
  return true;
}

int PFPacket::find_packet_start(uint8_t* buf, size_t buf_len)
{
  char p_type[2];
  get_type(p_type);
  for (size_t i = 0; i < buf_len - 4; i++)
  {
    if (((unsigned char)buf[i]) == 0x5c && ((unsigned char)buf[i + 1]) == 0xa2 &&
        ((unsigned char)buf[i + 2]) == p_type[0] && ((unsigned char)buf[i + 3]) == p_type[1])
    {
      return i;
    }
  }
  return -1;
}

void PFR2000Packet_A::read_with(PFPacketReader& reader)
{
  reader.read(*this);
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

void PFR2000Packet_B::read_with(PFPacketReader& reader)
{
  reader.read(*this);
}

void PFR2000Packet_B::read_data(uint8_t* buf, size_t num)
{
  Data* data = reinterpret_cast<Data*>(buf);
  distance.resize(num);
  amplitude.resize(num);
  for (int i = 0; i < num; i++)
  {
    distance[i] = data[i].distance;
    amplitude[i] = data[i].amplitude;
  }
}

void PFR2000Packet_C::read_with(PFPacketReader& reader)
{
  reader.read(*this);
}

void PFR2000Packet_C::read_data(uint8_t* buf, size_t num)
{
  uint32_t* data = reinterpret_cast<uint32_t*>(buf);
  distance.resize(num);
  amplitude.resize(num);
  for (int i = 0; i < num; i++)
  {
    uint32_t d = data[i];
    distance[i] = d & 0x000FFFFF;
    amplitude[i] = (d & 0xFFFFF000) >> 20;
  }
}

void PFR2300Packet_C1::read_with(PFPacketReader& reader)
{
  reader.read(*this);
}

void PFR2300Packet_C1::read_data(uint8_t* buf, size_t num)
{
  uint32_t* data = reinterpret_cast<uint32_t*>(buf);
  distance.resize(num);
  amplitude.resize(num);
  for (int i = 0; i < num; i++)
  {
    uint32_t d = data[i];
    distance[i] = d & 0x000FFFFF;
    amplitude[i] = (d & 0xFFFFF000) >> 20;
  }
}
