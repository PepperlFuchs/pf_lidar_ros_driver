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
#include "pf_driver/PFDriverR2000Config.h"
#include "pf_driver/PFDriverR2300Config.h"

class PFInterface
{
public:
  PFInterface() : state_(PFState::UNINIT)
  {
  }

  bool init(std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
            std::shared_ptr<ScanParameters> params, std::string topic, std::string frame_id);
  bool start_transmission();
  void stop_transmission();
  void terminate();

private:
  using PipelinePtr = std::unique_ptr<Pipeline<PFPacket>>;

  ros::NodeHandle nh_;
  ros::Timer watchdog_timer_;
  std::unique_ptr<Transport> transport_;
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

  std::shared_ptr<HandleInfo> info_;
  std::shared_ptr<ScanConfig> config_;
  std::shared_ptr<ScanParameters> params_;

  void change_state(PFState state);
  bool can_change_state(PFState state);
  bool handle_version(int major_version, int minor_version, std::string topic, std::string frame_id);

  void start_watchdog_timer(float duration);
  void feed_watchdog(const ros::TimerEvent& e);  // timer based

  PipelinePtr pipeline_;
  PipelinePtr get_pipeline(std::string packet_type);
  std::shared_ptr<Reader<PFPacket>> reader_;

  std::shared_ptr<std::mutex> config_mutex_;
  void on_shutdown();
};

#endif
