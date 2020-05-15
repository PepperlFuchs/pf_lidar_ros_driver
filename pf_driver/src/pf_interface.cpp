#include "pf_driver/pf/pf_interface.h"


bool PFInterface::init()
{
    // This is the first time ROS communicates with the device
    auto opi = protocol_interface_->get_protocol_info();
    if(opi.isError)
    {
        ROS_ERROR("Unable to communicate with device. Please check the IP address");
        return false;
    }
    // ROS_INFO("Info: %i %i %s %s", opi.version_major, opi.version_minor, opi.protocol_name.c_str(), opi.product.c_str());

    if(opi.protocol_name != "pfsdp")
        return false;
    if(!handle_version(opi.version_major, opi.version_minor))
        return false;

    change_state(PFState::INIT);
    return true;
}

void PFInterface::change_state(PFState state)
{
    if(state_ == state)
        return;
    state_ = state;     // Can use this function later
                        // to check state transitions
    std::string text;
    if(state_ == PFState::UNINIT)
        text = "Uninitialized";
    if(state_ == PFState::INIT)
        text = "Initialized";
    if(state_ == PFState::RUNNING)
        text = "Running";
    if(state_ == PFState::ERROR)
        text = "Error";
    ROS_INFO("Device state changed to %s", text.c_str());
}

bool PFInterface::can_change_state(PFState state)
{
    return true;
}

bool PFInterface::handle_version(int major_version, int minor_version)
{
    // std::cout << product_name << " " << product_name.find("R2000") << std::endl;
    std::string product_name = "product", expected = "expected";
    if(major_version == 1 && minor_version == 3)
    {
        protocol_interface_ = std::make_shared<PFSDP_2000>(ip_);
        expected = "R2000";
    }
    else if(major_version == 11 && minor_version == 31)
    {
        protocol_interface_ = std::make_shared<PFSDP_2300>(ip_);
        expected = "R2300";
    }

    product_name = protocol_interface_->get_product();
    if(product_name.find(expected) != std::string::npos)
    {
        ROS_INFO("Device found: %s", product_name.c_str());
        product_ = expected;
        return true;
    }
    ROS_ERROR("Device unsupported");
    return false;
}

bool PFInterface::connect()
{

    return true;
}

bool PFInterface::start_transmission()
{
    if(state_ != PFState::INIT)
        return false;

    HandleInfo info;
    if(transport_ == Connection::Transport::TCP)
    {
        if(port_.empty())
        {
            info = protocol_interface_->request_handle_tcp();
            port_ = info.port;
        }
        else
            info = protocol_interface_->request_handle_tcp(port_);
    }
    else if(transport_ == Connection::Transport::UDP)
    {
        // info = protocol_interface_->request_handle_udp();
    }
    ROS_INFO("%s %s", info.handle.c_str(), info.port.c_str());
    pipeline_ = get_pipeline(info.packet_type);
    if(!pipeline_->start())
        return false;

    protocol_interface_->start_scanoutput(info.handle);
    change_state(PFState::RUNNING);
    return true;
}

// What happens to the connection_ obj?
void PFInterface::stop_transmission()
{
    pipeline_->stop();
    pipeline_.reset();
    change_state(PFState::INIT);
}

std::shared_ptr<Pipeline<PFPacket>> PFInterface::get_pipeline(std::string packet_type)
{
    typedef PFPacket Packet;
    typedef PFPacketReader Reader_;
    if(product_ == "R2000")
    {
        if(packet_type == "A")
            typedef PFR2000Packet_A Packet;
        else if(packet_type == "B")
            typedef PFR2000Packet_B Packet;
        else if(packet_type == "C")
            typedef PFR2000Packet_C Packet;
        typedef PFR2000PacketReader Reader_;
    }
    else if(product_ == "R2300")
    {
        if(packet_type == "C1")
            typedef PFR2300Packet_C1 Packet;
        typedef PFR2300PacketReader Reader_;
    }
    auto parser = std::make_shared<PFParser<Packet>>();
    connection_->set_port(port_);
    std::shared_ptr<Writer<Packet>> writer = std::make_shared<PFWriter<Packet>>(std::move(connection_), parser);
    std::shared_ptr<Reader<Packet>> reader = std::make_shared<Reader_>();
    return std::make_shared<Pipeline<Packet>>(writer, reader);
}