#pragma once

#include "pf_driver/pf/pipeline.h"
#include "pf_driver/pf/pf_packet.h"

class PFPacketReader : public Reader<PFPacket>
{
public:
    virtual void read(std::shared_ptr<PFPacket> packet)
    {
    }

    virtual void start()
    {
    }

    virtual void stop()
    {
    }
};

class PFR2000PacketReader : public Reader<PFR2000Packet>
{
public:
    virtual void read(std::shared_ptr<PFR2000Packet> packet)
    {

    }

    // virtual void read(std::unique_ptr<PFR2000Packet_A> &packet);
    // virtual void read(std::unique_ptr<PFR2000Packet_B> &packet);
    // virtual void read(std::unique_ptr<PFR2000Packet_C> &packet);
};

class PFR2300PacketReader : public Reader<PFR2300Packet_C1>
{
public:
    //TODO: delete this. just for a test
    virtual void read(std::shared_ptr<PFR2300Packet_C1> packet)
    {
    }

    // virtual void read(std::unique_ptr<PFR2300Packet_C1> &packet)
    // {
    // }
};

