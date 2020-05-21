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

    virtual size_t get_size()
    {
        return ros::serialization::serializationLength(header);
    }

    virtual std::tuple<uint16_t, uint32_t, uint16_t> read_header(ros::serialization::IStream& stream)
    {
        ros::serialization::Serializer<pf_driver::PFHeader>::read(stream, header);
        return std::tuple<uint16_t, uint32_t, uint16_t>(header.header_size, header.packet_size, 0);
    }

    virtual void get_type(char *p_type)
    {
    }

    int find_packet_start(uint8_t* buf, size_t buf_len)
    {
        char p_type[2];
        get_type(p_type);
        for(size_t i = 0; i < buf_len - 4; i++)
        {
            if(((unsigned char) buf[i])   == 0x5c
            && ((unsigned char) buf[i+1]) == 0xa2
            && ((unsigned char) buf[i+2]) == p_type[0]
            && ((unsigned char) buf[i+3]) == p_type[1])
            {
                return i;
            }
        }
        return -1;
    }

    bool parse_buf(uint8_t* buf, size_t buf_len, size_t &remainder, size_t &p_size);
    virtual void read_with(PFPacketReader& reader){}
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
        return std::tuple<uint16_t, uint32_t, uint16_t>(header.header.header_size, header.header.packet_size, header.num_points_packet);
    }

    virtual void read_with(PFPacketReader& reader);

    pf_driver::PFR2000Header header;
};

class PFR2000Packet_A : public PFR2000Packet
{
public:
    // virtual void read_with(PFPacketReader& reader);
    virtual void get_type(char *c)
    {
        c[0] = 0x41;
        c[1] = 0x00;
    }
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

class PFR2300Packet : public PFPacket
{
public:
    std::vector<uint32_t> dist_amp;

};