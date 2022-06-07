#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include <rcutils/logging.h>
#include "pf_driver/pf/pf_interface.h"
#include "pf_driver/ros/scan_publisher.h"
#include "pf_driver/ros/hmi_image_listener.h"

bool PFInterface::init()
{
  protocol_interface_ = std::make_shared<PFSDPBase>(node_);
  if(!protocol_interface_->init())
  {
    return false;
  }

  //config_mutex_ = std::make_shared<std::mutex>();

  // This is the first time ROS communicates with the device
  auto opi = protocol_interface_->get_protocol_info();
  if (opi.isError)
  {
    RCLCPP_ERROR(node_->get_logger(), "Unable to communicate with device. Please check the IP address");
    return false;
  }

  if (opi.protocol_name != "pfsdp")
  {
    RCLCPP_ERROR(node_->get_logger(), "Incorrect protocol");
    return false;
  }

  if (!handle_version(opi.version_major, opi.version_minor))
  {
    RCLCPP_ERROR(node_->get_logger(), "Device unsupported");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Device found: %s", product_.c_str());

  auto info = protocol_interface_->get_handle_info();
  if (info->handle_type == HandleInfo::HANDLE_TYPE_UDP)
  {
    transport_ = std::make_unique<UDPTransport>(info->hostname);
    if (!transport_->connect())
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to establish UDP connection");
      return false;
    }

    info->endpoint = transport_->get_host_ip();
    info->port = transport_->get_port();
    protocol_interface_->request_handle_udp();
  }
  else if (info->handle_type == HandleInfo::HANDLE_TYPE_TCP)
  {
    transport_ = std::make_unique<TCPTransport>(info->hostname);
    protocol_interface_->request_handle_tcp();
    // if initially port was not set, request_handle sets it
    // set the updated port in transport
    transport_->set_port(info->port);
    transport_->connect();
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Incorrect transport option");
    return false;
  }

  if (info->handle.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not acquire communication handle");
    return false;
  }

  change_state(PFState::INIT);
  return true;
}

void PFInterface::change_state(PFState state)
{
  if (state_ == state)
    return;
  state_ = state;  // Can use this function later
                   // to check state transitions
  std::string text;
  if (state_ == PFState::UNINIT)
    text = "Uninitialized";
  if (state_ == PFState::INIT)
    text = "Initialized";
  if (state_ == PFState::RUNNING)
    text = "Running";
  if (state_ == PFState::SHUTDOWN)
    text = "Shutdown";
  if (state_ == PFState::ERROR)
    text = "Error";
  RCLCPP_INFO(node_->get_logger(), "Device state changed to %s", text.c_str());
}

bool PFInterface::can_change_state(PFState state)
{
  return true;
}

bool PFInterface::handle_version(int major_version, int minor_version)
{
  std::string expected_dev = "";
  if (major_version == 1 && minor_version == 4)
  {
    expected_dev = "R2000";
    protocol_interface_ = std::make_shared<PFSDP_2000>(protocol_interface_);
    reader_ = std::make_shared<ScanPublisherR2000>(
                               protocol_interface_->get_scan_config(),
                               protocol_interface_->get_scan_parameters(),
                               protocol_interface_->get_topic(),
                               protocol_interface_->get_frame_id(),
                               protocol_interface_->get_config_mutex(),
                               node_);
    hmi_listener_ = std::make_shared<HmiImageListener>(node_, protocol_interface_);
  }
  else if (major_version == 0 && minor_version == 5)
  {
    expected_dev = "R2300";
    protocol_interface_ = std::make_shared<PFSDP_2300>(protocol_interface_);
    reader_ = std::make_shared<ScanPublisherR2300>(
                               protocol_interface_->get_scan_config(),
                               protocol_interface_->get_scan_parameters(),
                               protocol_interface_->get_topic(),
                               protocol_interface_->get_frame_id(),
                               protocol_interface_->get_config_mutex(),
                               node_);
  }
  else
  {
    return false;
  }
  std::string product_name = protocol_interface_->get_product();
  if (product_name.find(expected_dev) != std::string::npos)
  {
    product_ = expected_dev;
    return true;
  }
  return false;
}

// TODO: this function needs a thorough clean-up
bool PFInterface::start_transmission()
{
  if (state_ != PFState::INIT)
    return false;

  if (pipeline_ && pipeline_->is_running())
    return true;

  pipeline_ = get_pipeline(protocol_interface_->get_scan_config()->packet_type);
  if (!pipeline_ || !pipeline_->start())
    return false;

  protocol_interface_->start_scanoutput();
  if (protocol_interface_->get_scan_config()->watchdog)
    start_watchdog_timer(std::chrono::milliseconds(protocol_interface_->get_scan_config()->watchdogtimeout));

  change_state(PFState::RUNNING);
  return true;
}

// What happens to the connection_ obj?
void PFInterface::stop_transmission()
{
  if (state_ != PFState::RUNNING)
    return;
  pipeline_->terminate();
  pipeline_.reset();
  protocol_interface_->stop_scanoutput(protocol_interface_->get_handle_info()->handle);
  change_state(PFState::SHUTDOWN);
}

void PFInterface::terminate()
{
  if (!pipeline_)
    return;
  pipeline_->terminate();
  pipeline_.reset();
}

std::unique_ptr<Pipeline<PFPacket>> PFInterface::get_pipeline(std::string packet_type)
{
  std::shared_ptr<Parser<PFPacket>> parser;
  std::shared_ptr<Writer<PFPacket>> writer;
  if (product_ == "R2000")
  {
    RCLCPP_DEBUG(node_->get_logger(), "PacketType is: %s", packet_type.c_str());
    if (packet_type == "A")
    {
      parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_A_Parser);
    }
    else if (packet_type == "B")
    {
      parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_B_Parser);
    }
    else if (packet_type == "C")
    {
      parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_C_Parser);
    }
  }
  else if (product_ == "R2300")
  {
    if (packet_type == "C1")
    {
      parser = std::unique_ptr<Parser<PFPacket>>(new PFR2300_C1_Parser);
    }
  }
  if (!parser)
  {
    return nullptr;
  }
  writer = std::shared_ptr<Writer<PFPacket>>(new PFWriter<PFPacket>(std::move(transport_), parser, node_->get_logger()));
  return std::unique_ptr<Pipeline<PFPacket>>(
      new Pipeline<PFPacket>(writer, reader_, std::bind(&PFInterface::on_shutdown, this), node_->get_logger()));
}

void PFInterface::start_watchdog_timer(std::chrono::milliseconds duration)
{
  std::chrono::duration feed_time = std::min(duration, std::chrono::milliseconds(std::chrono::seconds(60)));
  watchdog_timer_ = node_->create_wall_timer(feed_time, std::bind(&PFInterface::feed_watchdog, this));
}

void PFInterface::feed_watchdog()
{
  protocol_interface_->feed_watchdog();
}

void PFInterface::on_shutdown()
{
  RCLCPP_INFO(node_->get_logger(), "Shutting down pipeline!");
  stop_transmission();
}
