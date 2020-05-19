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

    virtual size_t get_size()
    {
        return ros::serialization::serializationLength(header);
    }

    virtual void read(ros::serialization::IStream stream)
    {
        ros::serialization::Serializer<pf_driver::PFHeader>::read(stream, header);
    }

    bool parse_buf(uint8_t* buf, size_t buf_len);
    virtual void read_with(PFPacketReader& reader){}
};

class PFR2000Packet : public PFPacket
{
public:
    virtual size_t get_size()
    {
        return ros::serialization::serializationLength(header);
    }

    virtual void read(ros::serialization::IStream stream)
    {
        ros::serialization::Serializer<pf_driver::PFR2000Header>::read(stream, header);
    }

    pf_driver::PFR2000Header header;
};

class PFR2000Packet_A : public PFR2000Packet
{
public:
    std::vector<std::uint32_t> distance;        // measured distance (mm)

    virtual void read_with(PFPacketReader& reader);
};

class PFR2000Packet_B : public PFR2000Packet
{
public:
    std::vector<std::uint32_t> distance;        // measured distance (mm)
    std::vector<std::uint16_t> amplitude;       // measured amplitude (mm)
    std::vector<std::uint8_t> payload_padding;  // 0 or 2 bytes padding
};

class PFR2000Packet_C : public PFR2000Packet
{
public:
    std::vector<std::uint32_t> dist_amp;        //TODO: need to check this
};

class PFR2300Packet_C1 : public PFPacket
{
public:
    std::vector<uint32_t> dist_amp;

};