#ifndef PF_DRIVER_PF_INTERFACE_H
#define PF_DRIVER_PF_INTERFACE_H

#pragma once

#include <string>
#include <memory>

#include "pf_driver/pf/reader.h"
#include "pf_driver/pf/writer.h"
#include "pf_driver/pf/pf_parser.h"
#include "pf_driver/communication.h"
#include "pf_driver/pf/r2000/pfsdp_2000.hpp"
#include "pf_driver/pf/r2300/pfsdp_2300.hpp"

class PFInterface
{

public:
    PFInterface(std::unique_ptr<Connection> &&connection) : connection_(std::move(connection))
    {
        state_ = PFState::UNINIT;
        if(connection_)
        {
            ip_ = connection_->get_device_ip();
            transport_ = connection_->TRANSPORT;   
        }
        protocol_interface_ = std::make_shared<PFSDPBase>(ip_);
    }

    bool init();
    bool connect();
    bool start_transmission();
    void stop_transmission();

private:
    std::string ip_, port_;
    std::unique_ptr<Connection> connection_;
    Connection::Transport transport_;
    std::shared_ptr<PFSDPBase> protocol_interface_;

    enum class PFState
    {
        UNINIT,
        INIT,
        RUNNING,
        ERROR
    };
    PFState state_;
    std::string product_;

    void change_state(PFState state);
    bool can_change_state(PFState state);
    bool handle_version(int major_version, int minor_version);

    std::shared_ptr<Pipeline<PFPacket>> pipeline_;
    std::shared_ptr<Pipeline<PFPacket>> get_pipeline(std::string packet_type);
};

#endif