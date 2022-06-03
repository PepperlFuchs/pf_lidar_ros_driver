#ifndef PF_DRIVER_PF_INTERFACE_H
#define PF_DRIVER_PF_INTERFACE_H

#pragma once

#include <string>
#include <memory>
#include <future>

#include <rclcpp/logger.hpp>
#include "pf_driver/pf/pf_parser.h"
#include "pf_driver/pf/writer.h"
#include "pf_driver/pf/reader.h"
#include "pf_driver/communication.h"
#include "pf_driver/pf/r2000/pfsdp_2000.hpp"
#include "pf_driver/pf/r2300/pfsdp_2300.hpp"

class PFInterface
{
public:
  PFInterface(std::shared_ptr<rclcpp::Node> node) : node_(node), state_(PFState::UNINIT)
  {
  }

  bool init();
  bool start_transmission();
  void stop_transmission();
  void terminate();

private:
  using PipelinePtr = std::unique_ptr<Pipeline<PFPacket>>;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
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

  void change_state(PFState state);
  bool can_change_state(PFState state);
  bool handle_version(int major_version, int minor_version);

  void start_watchdog_timer(std::chrono::milliseconds duration);
  void feed_watchdog();  // timer based

  PipelinePtr pipeline_;
  PipelinePtr get_pipeline(std::string packet_type);
  std::shared_ptr<Reader<PFPacket>> reader_;

  //std::shared_ptr<std::mutex> config_mutex_;
  void on_shutdown();
};

#endif
