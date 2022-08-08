#pragma once

#include <vector>
#include <ros/serialization.h>
#include "pf_driver/PFR2000Header.h"
#include "pf_driver/PFR2300Header.h"

class PFPacketReader;

class PFPacket
{
public:
  pf_driver::PFHeader header;
  std::vector<uint32_t> distance;
  std::vector<uint16_t> amplitude;

  virtual void read_with(PFPacketReader& reader)
  {
  }

  int find_packet_start(uint8_t* buf, size_t buf_len);
  bool parse_buf(uint8_t* buf, size_t buf_len, size_t& remainder, size_t& p_size);

protected:
  virtual size_t get_size() = 0;
  virtual void get_type(char* p_type) = 0;
  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(ros::serialization::IStream& stream) = 0;
  virtual void read_data(uint8_t* buf, size_t num) = 0;
};

class PFR2000Packet : public PFPacket
{
public:
  virtual size_t get_size()
  {
    return ros::serialization::serializationLength(header);
  }

  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(ros::serialization::IStream& stream)
  {
    ros::serialization::Serializer<pf_driver::PFR2000Header>::read(stream, header);
    return std::tuple<uint16_t, uint32_t, uint16_t>(header.header.header_size, header.header.packet_size,
                                                    header.num_points_packet);
  }

  pf_driver::PFR2000Header header;
};

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

  virtual void get_type(char* c)
  {
    c[0] = 0x42;
    c[1] = 0x00;
  }

  virtual void read_data(uint8_t* buf, size_t num);
  virtual void read_with(PFPacketReader& reader);
};

class PFR2000Packet_C : public PFR2000Packet
{
protected:
#pragma pack(push, pfC, 1)
  struct Data
  {
    uint32_t dist_amp;
  };
#pragma pack(pop, pfC)

  virtual void get_type(char* c)
  {
    c[0] = 0x43;
    c[1] = 0x00;
  }

  virtual void read_data(uint8_t* buf, size_t num);
  virtual void read_with(PFPacketReader& reader);
};

class PFR2300Packet : public PFPacket
{
public:
  virtual size_t get_size()
  {
    return ros::serialization::serializationLength(header);
  }

  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(ros::serialization::IStream& stream)
  {
    ros::serialization::Serializer<pf_driver::PFR2300Header>::read(stream, header);
    return std::tuple<uint16_t, uint32_t, uint16_t>(header.header.header_size, header.header.packet_size,
                                                    header.num_points_packet);
  }

  pf_driver::PFR2300Header header;
};

class PFR2300Packet_C1 : public PFR2300Packet
{
protected:
#pragma pack(push, pfC1, 1)
  struct Data
  {
    uint32_t dist_amp;
  };
#pragma pack(pop, pfC1)

  virtual void get_type(char* c)
  {
    c[0] = 0x43;
    c[1] = 0x31;
  }

  virtual void read_data(uint8_t* buf, size_t num);
  virtual void read_with(PFPacketReader& reader);
};