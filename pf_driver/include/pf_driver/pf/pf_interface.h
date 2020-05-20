#ifndef PF_DRIVER_PF_INTERFACE_H
#define PF_DRIVER_PF_INTERFACE_H

#pragma once

#include <string>
#include <memory>
#include <future>

#include "pf_driver/pf/pf_parser.h"
#include "pf_driver/pf/writer.h"
#include "pf_driver/pf/reader.h"
#include "pf_driver/communication.h"
#include "pf_driver/pf/r2000/pfsdp_2000.hpp"
#include "pf_driver/pf/r2300/pfsdp_2300.hpp"

class PFInterface
{

public:
    PFInterface(std::shared_ptr<Connection> connection) : connection_(connection), state_(PFState::UNINIT)
    {
        if(connection_)
        {
            ip_ = connection_->get_device_ip();
            transport_ = connection_->TRANSPORT;   
        }
        protocol_interface_ = std::make_shared<PFSDPBase>(ip_);
    }

    PFInterface(Connection::Transport transport, std::string IP) : ip_(IP), transport_(transport), state_(PFState::UNINIT)
    {
        ip_ = IP;
        transport_ = transport;
        protocol_interface_ = std::make_shared<PFSDPBase>(ip_);
    }

    bool init();
    bool start_transmission();
    void stop_transmission();
    void terminate();

private:
    using PipelinePtr = std::unique_ptr<Pipeline<PFPacket>>;

    ros::NodeHandle nh_;
    std::string ip_, port_;
    ros::Timer watchdog_timer_;
    std::shared_ptr<Connection> connection_;
    Connection::Transport transport_;
    std::shared_ptr<PFSDPBase> protocol_interface_;

    enum class PFState
    {
        UNINIT,
        INIT,
        RUNNING,
        SHUTDOWN,
        ERROR
    };
    PFState state_;
    std::string product_;

    HandleInfo info_;
    ScanConfig config_;
    ScanParameters params_;

    void change_state(PFState state);
    bool can_change_state(PFState state);
    bool handle_version(int major_version, int minor_version);

    void start_watchdog_timer(float duration);
    void feed_watchdog(const ros::TimerEvent& e);   //timer based

    PipelinePtr pipeline_;
    PipelinePtr get_pipeline(std::string packet_type);

    std::mutex mutex_;
    void on_shutdown();
};

#endif