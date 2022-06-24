#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "pf_driver/pf/pf_interface.h"
#include "pf_driver/ros/scan_publisher.h"

bool PFInterface::init(std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
                       std::shared_ptr<ScanParameters> params, std::string topic, std::string frame_id,
                       const uint16_t num_layers)
{
  config_ = config;
  info_ = info;
  params_ = params;

  config_mutex_ = std::make_shared<std::mutex>();

  protocol_interface_ = std::make_shared<PFSDPBase>(info, config, params, config_mutex_);
  // This is the first time ROS communicates with the device
  auto opi = protocol_interface_->get_protocol_info();
  if (opi.isError)
  {
    ROS_ERROR("Unable to communicate with device. Please check the IP address");
    return false;
  }

  if (opi.protocol_name != "pfsdp")
  {
    ROS_ERROR("Incorrect protocol");
    return false;
  }

  if (!handle_version(opi.version_major, opi.version_minor, opi.device_family, topic, frame_id, num_layers))
  {
    ROS_ERROR("Device unsupported");
    return false;
  }
  ROS_INFO("Device found: %s", product_.c_str());

  if (info->handle_type == HandleInfo::HANDLE_TYPE_UDP)
  {
    transport_ = std::make_unique<UDPTransport>(info->hostname);
    if (!transport_->connect())
    {
      ROS_ERROR("Unable to establish UDP connection");
      return false;
    }

    info_->endpoint = transport_->get_host_ip();
    info_->port = transport_->get_port();
    protocol_interface_->request_handle_udp();
  }
  else if (info->handle_type == HandleInfo::HANDLE_TYPE_TCP)
  {
    transport_ = std::make_unique<TCPTransport>(info->hostname);
    protocol_interface_->request_handle_tcp();
    // if initially port was not set, request_handle sets it
    // set the updated port in transport
    transport_->set_port(info_->port);
    transport_->connect();
  }
  else
  {
    ROS_ERROR("Incorrect transport option");
    return false;
  }

  if (info_->handle.empty())
  {
    ROS_ERROR("Could not acquire communication handle");
    return false;
  }

  protocol_interface_->setup_param_server();
  //   protocol_interface_->update_scanoutput_config();
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
  ROS_INFO("Device state changed to %s", text.c_str());
}

bool PFInterface::can_change_state(PFState state)
{
  return true;
}

bool PFInterface::start_transmission()
{
  if (state_ != PFState::INIT)
    return false;

  if (pipeline_ && pipeline_->is_running())
    return true;

  pipeline_ = get_pipeline(config_->packet_type);
  if (!pipeline_ || !pipeline_->start())
    return false;

  protocol_interface_->start_scanoutput();
  if (config_->watchdog)
    start_watchdog_timer(config_->watchdogtimeout / 1000.0);

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
  protocol_interface_->stop_scanoutput(info_->handle);
  change_state(PFState::SHUTDOWN);
}

void PFInterface::terminate()
{
  if (!pipeline_)
    return;
  pipeline_->terminate();
  pipeline_.reset();
}

void PFInterface::start_watchdog_timer(float duration)
{
  int feed_time = std::floor(std::min(duration, 60.0f));
  watchdog_timer_ =
      nh_.createTimer(ros::Duration(feed_time), std::bind(&PFInterface::feed_watchdog, this, std::placeholders::_1));
}

void PFInterface::feed_watchdog(const ros::TimerEvent& e)
{
  protocol_interface_->feed_watchdog(info_->handle);
}

void PFInterface::on_shutdown()
{
  ROS_INFO("Shutting down pipeline!");
  stop_transmission();
}

// factory functions
bool PFInterface::handle_version(int major_version, int minor_version, int device_family, std::string topic,
                                 std::string frame_id, const uint16_t num_layers)
{
  std::string expected_dev = "";
  if (device_family == 1 || device_family == 3 || device_family == 6)
  {
    expected_dev = "R2000";
    protocol_interface_ = std::make_shared<PFSDP_2000>(info_, config_, params_, config_mutex_);
    reader_ = std::shared_ptr<PFPacketReader>(
        new LaserscanPublisher(config_, params_, topic.c_str(), frame_id.c_str(), config_mutex_));
  }
  else if (device_family == 5 || device_family == 7)
  {
    expected_dev = "R2300";
    protocol_interface_ = std::make_shared<PFSDP_2300>(info_, config_, params_, config_mutex_);

    if (device_family == 5)
    {
      std::string part = protocol_interface_->get_part();
      reader_ = std::shared_ptr<PFPacketReader>(new PointcloudPublisher(
          config_, params_, topic.c_str(), frame_id.c_str(), config_mutex_, num_layers, part.c_str()));
    }
    else if (device_family == 7)
    {
      reader_ = std::shared_ptr<PFPacketReader>(
          new LaserscanPublisher(config_, params_, topic.c_str(), frame_id.c_str(), config_mutex_));
    }
    else
    {
      return false;
    }
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

std::unique_ptr<Pipeline<PFPacket>> PFInterface::get_pipeline(std::string packet_type)
{
  std::shared_ptr<Parser<PFPacket>> parser;
  std::shared_ptr<Writer<PFPacket>> writer;
  if (product_ == "R2000")
  {
    ROS_DEBUG("PacketType is: %s", packet_type.c_str());
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
  writer = std::shared_ptr<Writer<PFPacket>>(new PFWriter<PFPacket>(std::move(transport_), parser));
  return std::unique_ptr<Pipeline<PFPacket>>(
      new Pipeline<PFPacket>(writer, reader_, std::bind(&PFInterface::on_shutdown, this)));
}
