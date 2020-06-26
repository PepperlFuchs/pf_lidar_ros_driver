#ifndef PF_DRIVER_PF_INTERFACE_H
#define PF_DRIVER_PF_INTERFACE_H

#pragma once

#include <string>
#include <memory>
#include <future>
#include <dynamic_reconfigure/server.h>

#include "pf_driver/pf/pf_parser.h"
#include "pf_driver/pf/writer.h"
#include "pf_driver/pf/reader.h"
#include "pf_driver/communication.h"
#include "pf_driver/pf/r2000/pfsdp_2000.hpp"
#include "pf_driver/pf/r2300/pfsdp_2300.hpp"
#include "pf_driver/PFDriverConfig.h"

class PFInterface
{

public:
    PFInterface(std::unique_ptr<Transport> &&transport, std::string device) : transport_(std::move(transport)), state_(PFState::UNINIT), expected_device_(device)
    {
        if(transport_)
        {
            ip_ = transport_->get_device_ip();
            transport_type_ = transport_->get_type();   
        }
        protocol_interface_ = std::make_shared<PFSDPBase>(ip_);

        dynamic_reconfigure::Server<pf_driver::PFDriverConfig> param_server;
        param_server.setCallback(boost::bind(&PFInterface::reconfig_callback, this, boost::placeholders::_1, boost::placeholders::_2));
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
    std::unique_ptr<Transport> transport_;
    transport_type transport_type_;
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
    std::string expected_device_;

    HandleInfo info_;
    ScanConfig config_;
    ScanParameters params_;

    void change_state(PFState state);
    bool can_change_state(PFState state);
    bool handle_version(int major_version, int minor_version);

    void start_watchdog_timer(float duration);
    void feed_watchdog(const ros::TimerEvent& e);   //timer based
    void reconfig_callback(pf_driver::PFDriverConfig &config, uint32_t level);

    PipelinePtr pipeline_;
    PipelinePtr get_pipeline(std::string packet_type);

    std::mutex mutex_;
    void on_shutdown();
};

#endif