#pragma once

#include <vector>
#include <rclcpp/serialization.hpp>
#include "pf_interfaces/msg/pfr2000_header.hpp"
#include "pf_interfaces/msg/pfr2300_header.hpp"

class PFPacketReader;

struct PFHeaderMsg
{
  // Although this is a very effective method of parsing data, it does not handle platforms when integers endianness
  // is not the same as the one sent by the scanner
  #pragma pack(push, pfHeader, 1)
  struct
  {
    uint16_t magic;
    uint16_t packet_type;
    uint32_t packet_size;
    uint16_t header_size;
    uint16_t scan_number;
    uint16_t packet_number;
  } header_common;
  union
  {
    struct
    {
      uint64_t timestamp_raw;
      uint64_t timestamp_sync;
      uint32_t status_flags;
      uint32_t scan_frequency;
      uint16_t num_points_scan;
      uint16_t num_points_packet;
      uint16_t first_index;
      int32_t first_angle;
      int32_t angular_increment;
      uint32_t iq_input;
      uint32_t iq_overload;
      uint64_t iq_timestamp_raw;
      uint64_t iq_timestamp_sync;
    } r2000;
    struct
    {
      uint16_t layer_index;
      int32_t layer_inclination;
      uint64_t timestamp_raw;
      uint64_t reserved1;
      uint32_t status_flags;
      uint32_t scan_frequency;
      uint16_t num_points_scan;
      uint16_t num_points_packet;
      uint16_t first_index;
      int32_t first_angle;
      int32_t angular_increment;
      uint32_t reserved2;
      uint32_t reserved3;
      uint64_t reserved4;
      uint64_t reserved5;
    } r2300;
  } header_specific;
  #pragma pack(pop, pfHeader)
};

class PFPacket
{
public:
  pf_interfaces::msg::PFHeader header;
  std::vector<uint32_t> distance;
  std::vector<uint16_t> amplitude;

  virtual void read_with(PFPacketReader& reader)
  {
  }

  int find_packet_start(uint8_t* buf, size_t buf_len);
  bool parse_buf(uint8_t* buf, size_t buf_len, size_t& remainder, size_t& p_size);

protected:
  std::tuple<uint16_t, uint32_t> read_header_common(uint8_t* buf)
  {
    PFHeaderMsg *msg = reinterpret_cast<PFHeaderMsg *>(buf);
    return std::tuple<uint16_t, uint32_t>(msg->header_common.header_size, msg->header_common.packet_size);
  }

  virtual size_t get_size() = 0;
  virtual void get_type(char* p_type) = 0;
  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(uint8_t* buf) = 0;
  virtual void read_data(uint8_t* buf, size_t num) = 0;
};

class PFR2000Packet : public PFPacket
{
public:
  virtual size_t get_size() override
  {
    // See https://answers.ros.org/question/303992/how-to-get-the-serialized-message-sizelength-in-ros2/
    return 14 + 62;
  }

  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(uint8_t* buf) override
  {
    auto header_common = read_header_common(buf);

    PFHeaderMsg *msg = reinterpret_cast<PFHeaderMsg *>(buf);

    header.header.magic = msg->header_common.magic;
    header.header.packet_type = msg->header_common.packet_type;
    header.header.packet_size = msg->header_common.packet_size;
    header.header.header_size = msg->header_common.header_size;
    header.header.scan_number = msg->header_common.scan_number;
    header.header.packet_number = msg->header_common.packet_number;

    header.timestamp_raw = msg->header_specific.r2000.timestamp_raw;
    header.timestamp_sync = msg->header_specific.r2000.timestamp_sync;
    header.status_flags = msg->header_specific.r2000.status_flags;
    header.scan_frequency = msg->header_specific.r2000.scan_frequency;
    header.num_points_scan = msg->header_specific.r2000.num_points_scan;
    header.num_points_packet = msg->header_specific.r2000.num_points_packet;
    header.first_index = msg->header_specific.r2000.first_index;
    header.first_angle = msg->header_specific.r2000.first_angle;
    header.angular_increment = msg->header_specific.r2000.angular_increment;
    header.iq_input = msg->header_specific.r2000.iq_input;
    header.iq_overload = msg->header_specific.r2000.iq_overload;
    header.iq_timestamp_raw = msg->header_specific.r2000.iq_timestamp_raw;
    header.iq_timestamp_sync = msg->header_specific.r2000.iq_timestamp_sync;

    return std::tuple<uint16_t, uint32_t, uint16_t>(std::get<0>(header_common),
                                                    std::get<1>(header_common),
                                                    msg->header_specific.r2000.num_points_packet);
  }

  pf_interfaces::msg::PFR2000Header header;
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

  virtual void get_type(char* c) override
  {
    c[0] = 0x41;
    c[1] = 0x00;
  }

  virtual void read_data(uint8_t* buf, size_t num) override;
  virtual void read_with(PFPacketReader& reader) override;
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

  virtual void read_data(uint8_t* buf, size_t num) override;
  virtual void read_with(PFPacketReader& reader) override;
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

  virtual void read_data(uint8_t* buf, size_t num) override;
  virtual void read_with(PFPacketReader& reader) override;
};

class PFR2300Packet : public PFPacket
{
public:
  virtual size_t get_size() override
  {
    // See https://answers.ros.org/question/303992/how-to-get-the-serialized-message-sizelength-in-ros2/
    return 14 + 68;
  }

  virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(uint8_t* buf) override
  {
    auto header_common = read_header_common(buf);

    PFHeaderMsg *msg = reinterpret_cast<PFHeaderMsg *>(buf);

    header.header.magic = msg->header_common.magic;
    header.header.packet_type = msg->header_common.packet_type;
    header.header.packet_size = msg->header_common.packet_size;
    header.header.header_size = msg->header_common.header_size;
    header.header.scan_number = msg->header_common.scan_number;
    header.header.packet_number = msg->header_common.packet_number;

    uint16_t layer_index;
    int32_t layer_inclination;
    uint64_t timestamp_raw;
    uint64_t reserved1;
    uint32_t status_flags;
    uint32_t scan_frequency;
    uint16_t num_points_scan;
    uint16_t num_points_packet;
    uint16_t first_index;
    int32_t first_angle;
    int32_t angular_increment;
    uint32_t reserved2;
    uint32_t reserved3;
    uint64_t reserved4;
    uint64_t reserved5;

    header.layer_index = msg->header_specific.r2300.layer_index;
    header.layer_inclination = msg->header_specific.r2300.layer_inclination;
    header.timestamp_raw = msg->header_specific.r2300.timestamp_raw;
    header.reserved1 = msg->header_specific.r2300.reserved1;
    header.status_flags = msg->header_specific.r2300.status_flags;
    header.scan_frequency = msg->header_specific.r2300.scan_frequency;
    header.num_points_scan = msg->header_specific.r2300.num_points_scan;
    header.num_points_packet = msg->header_specific.r2300.num_points_packet;
    header.first_index = msg->header_specific.r2300.first_index;
    header.first_angle = msg->header_specific.r2300.first_angle;
    header.angular_increment = msg->header_specific.r2300.angular_increment;
    header.reserved2 = msg->header_specific.r2300.reserved2;
    header.reserved3 = msg->header_specific.r2300.reserved3;
    header.reserved4 = msg->header_specific.r2300.reserved4;
    header.reserved5 = msg->header_specific.r2300.reserved5;

    return std::tuple<uint16_t, uint32_t, uint16_t>(std::get<0>(header_common),
                                                    std::get<1>(header_common),
                                                    msg->header_specific.r2300.num_points_packet);
  }

  pf_interfaces::msg::PFR2300Header header;
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

  virtual void get_type(char* c) override
  {
    c[0] = 0x43;
    c[1] = 0x31;
  }

  virtual void read_data(uint8_t* buf, size_t num) override;
  virtual void read_with(PFPacketReader& reader) override;
};
