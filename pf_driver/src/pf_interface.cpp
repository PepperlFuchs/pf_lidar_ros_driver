#include "pf_driver/pf/pf_interface.h"
#include "pf_driver/ros/scan_publisher.h"


bool PFInterface::init()
{
    // This is the first time ROS communicates with the device
    auto opi = protocol_interface_->get_protocol_info();
    if(opi.isError)
    {
        ROS_ERROR("Unable to communicate with device. Please check the IP address");
        return false;
    }
    // ROS_INFO("Info: %i %i %s", opi.version_major, opi.version_minor, opi.protocol_name.c_str());

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
    if(state_ == PFState::SHUTDOWN)
        text = "Shutdown";
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
    std::string product_name = "product", expected = "expected";
    if(major_version == 1 && minor_version == 3)
    {
        protocol_interface_ = std::make_shared<PFSDP_2000>(ip_);
        expected = "R2000";
    }
    else if(major_version == 0 && minor_version == 5)
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

bool PFInterface::start_transmission()
{
    if(state_ != PFState::INIT)
        return false;

    if(pipeline_ && pipeline_->is_running())
        return true;

    if(transport_ == Connection::Transport::TCP)
    {
        if(port_.empty())
        {
            info_ = protocol_interface_->request_handle_tcp();
            port_ = info_.port;
        }
        else
        {
            info_ = protocol_interface_->request_handle_tcp(port_);
        }
        connection_->set_port(port_);
    }
    else if(transport_ == Connection::Transport::UDP)
    {
        if(!connection_->connect())
            return false;

        std::string host_ip = connection_->get_host_ip();
        port_ = connection_->get_port();
        info_ = protocol_interface_->request_handle_udp(host_ip, port_);
    }
    config_ = protocol_interface_->get_scanoutput_config(info_.handle);
    params_ = protocol_interface_->get_scan_parameters(config_.start_angle);

    config_.print();
    params_.print();

    pipeline_ = get_pipeline(config_.packet_type);
    pipeline_->set_scanoutput_config(config_);
    pipeline_->set_scan_params(params_);

    if(!pipeline_->start())
        return false;

    

    protocol_interface_->start_scanoutput(info_.handle);
    if(config_.watchdog)
        start_watchdog_timer(config_.watchdogtimeout / 1000.0);

    change_state(PFState::RUNNING);
    return true;
}

// What happens to the connection_ obj?
void PFInterface::stop_transmission()
{
    if(state_ != PFState::RUNNING)
        return;
    pipeline_->terminate();
    pipeline_.reset();
    protocol_interface_->stop_scanoutput(info_.handle);
    change_state(PFState::SHUTDOWN);
}

void PFInterface::terminate()
{
    if(!pipeline_)
        return;
    pipeline_->terminate();
    pipeline_.reset();
}

std::unique_ptr<Pipeline<PFPacket>> PFInterface::get_pipeline(std::string packet_type)
{
    std::shared_ptr<Parser<PFPacket>> parser;
    std::shared_ptr<Writer<PFPacket>> writer;
    std::shared_ptr<Reader<PFPacket>> reader;
    if(product_ == "R2000")
    {
        if(packet_type == "A")
        {
            parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_A_Parser);
        }
        else if(packet_type == "B")
        {
            parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_B_Parser);
        }
        else if(packet_type == "C")
        {
            parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_C_Parser);
        }
    }
    else if(product_ == "R2300")
    {
        if(packet_type == "C1")
        {
            parser = std::unique_ptr<Parser<PFPacket>>(new PFR2300_C1_Parser);
        }
    }
    writer = std::shared_ptr<Writer<PFPacket>>(new PFWriter<PFPacket>(connection_, parser));
    reader = std::shared_ptr<Reader<PFPacket>>(new ScanPublisher("/scan", "scanner"));
    return std::unique_ptr<Pipeline<PFPacket>>(new Pipeline<PFPacket>(writer, reader, std::bind(&PFInterface::on_shutdown, this)));
}

void PFInterface::start_watchdog_timer(float duration)
{
    int feed_time = std::floor(std::min(duration, 60.0f));
    watchdog_timer_ = nh_.createTimer(ros::Duration(feed_time), std::bind(&PFInterface::feed_watchdog, this, std::placeholders::_1));
}

void PFInterface::feed_watchdog(const ros::TimerEvent& e)
{
    protocol_interface_->feed_watchdog(info_.handle);
}

void PFInterface::on_shutdown()
{
    ROS_INFO("Shutting down pipeline!");
    stop_transmission();
}
