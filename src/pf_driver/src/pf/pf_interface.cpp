#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include <rcutils/logging.h>
#include "pf_driver/pf/pf_interface.h"
#include "pf_driver/ros/laser_scan_publisher.h"
#include "pf_driver/ros/point_cloud_publisher.h"
#include "pf_driver/communication/udp_transport.h"
#include "pf_driver/communication/tcp_transport.h"

PFInterface::PFInterface(std::shared_ptr<rclcpp::Node> node) : node_(node), state_(PFState::UNINIT)
{
}

bool PFInterface::init(std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
                       std::shared_ptr<ScanParameters> params, const std::string& topic, const std::string& frame_id,
                       const uint16_t num_layers)
{
  config_ = config;
  info_ = info;
  params_ = params;

  topic_ = topic;
  frame_id_ = frame_id;
  num_layers_ = num_layers;

  protocol_interface_ = std::make_shared<PFSDPBase>(node_, info, config, params);
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

  if (!handle_version(opi.version_major, opi.version_minor, opi.device_family, topic, frame_id, num_layers))
  {
    RCLCPP_ERROR(node_->get_logger(), "Device unsupported");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Device found: %s", product_.c_str());

  // release previous handles
  if (!prev_handle_.empty())
  {
    protocol_interface_->release_handle(prev_handle_);
  }

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
    transport_->set_port(info_->port);
    if (!transport_->connect())
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to establish TCP connection");
      return false;
    }
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

  prev_handle_ = info_->handle;
  protocol_interface_->setup_parameters_callback();
  protocol_interface_->update_scanoutput_config();
  protocol_interface_->set_connection_failure_cb(std::bind(&PFInterface::connection_failure_cb, this));
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

bool PFInterface::start_transmission(std::shared_ptr<std::mutex> net_mtx,
                                     std::shared_ptr<std::condition_variable> net_cv, bool& net_fail)
{
  if (state_ != PFState::INIT)
    return false;

  if (pipeline_ && pipeline_->is_running())
    return true;

  pipeline_ = get_pipeline(config_->packet_type, net_mtx, net_cv, net_fail);
  if (!pipeline_ || !pipeline_->start())
    return false;

  protocol_interface_->start_scanoutput();
  // start watchdog timer is watchdog is true in config AND if the device is not R2000
  // since watchdog for R2300 is always off
  if (config_->watchdog && protocol_interface_->get_product().find("R2000") != std::string::npos)
  {
    start_watchdog_timer(config_->watchdogtimeout / 1000.0);
  }
  change_state(PFState::RUNNING);
  return true;
}

// What happens to the connection_ obj?
void PFInterface::stop_transmission()
{
  if (state_ != PFState::RUNNING)
    return;
  protocol_interface_->stop_scanoutput(info_->handle);
  protocol_interface_->release_handle(info_->handle);
  change_state(PFState::INIT);
}

void PFInterface::terminate()
{
  if (!pipeline_)
    return;

  if (config_->watchdog)
  {
    watchdog_timer_->cancel();
  }

  pipeline_->terminate();
  pipeline_.reset();
  protocol_interface_.reset();
  transport_.reset();
  change_state(PFState::UNINIT);
}

bool PFInterface::init()
{
  return init(info_, config_, params_, topic_, frame_id_, num_layers_);
}

void PFInterface::start_watchdog_timer(float duration)
{
  // dividing the watchdogtimeout by 2 to have a “safe” feed time within the defined timeout
  float feed_time_sec = std::min(duration, 60.0f) / 2.0f;
  int feed_time = feed_time_sec * 1000;
  watchdog_timer_ =
      node_->create_wall_timer(std::chrono::milliseconds(feed_time), std::bind(&PFInterface::feed_watchdog, this));
}

void PFInterface::feed_watchdog()
{
  protocol_interface_->feed_watchdog(info_->handle);
}

void PFInterface::on_shutdown()
{
  RCLCPP_INFO(node_->get_logger(), "Shutting down pipeline!");
  // stop_transmission();
}

void PFInterface::connection_failure_cb()
{
  std::cout << "handling connection failure" << std::endl;
  terminate();
  std::cout << "terminated" << std::endl;
  while (!init())
  {
    std::cout << "trying to reconnect..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

// factory functions
bool PFInterface::handle_version(int major_version, int minor_version, int device_family, const std::string& topic,
                                 const std::string& frame_id, const uint16_t num_layers)
{
  std::string expected_dev = "";
  if (device_family == 1 || device_family == 3 || device_family == 6)
  {
    expected_dev = "R2000";
    protocol_interface_ = std::make_shared<PFSDP_2000>(node_, info_, config_, params_);
    reader_ = std::shared_ptr<PFPacketReader>(
        new LaserscanPublisher(node_, config_, params_, topic.c_str(), frame_id.c_str()));
  }
  else if (device_family == 5 || device_family == 7)
  {
    expected_dev = "R2300";
    protocol_interface_ = std::make_shared<PFSDP_2300>(node_, info_, config_, params_);

    if (device_family == 5)
    {
      std::string part = protocol_interface_->get_part();
      reader_ = std::shared_ptr<PFPacketReader>(
          new PointcloudPublisher(node_, config_, params_, topic.c_str(), frame_id.c_str(), num_layers, part.c_str()));
    }
    else if (device_family == 7)
    {
      reader_ = std::shared_ptr<PFPacketReader>(
          new LaserscanPublisher(node_, config_, params_, topic.c_str(), frame_id.c_str()));
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

std::unique_ptr<Pipeline> PFInterface::get_pipeline(const std::string& packet_type, std::shared_ptr<std::mutex> net_mtx,
                                                    std::shared_ptr<std::condition_variable> net_cv, bool& net_fail)
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
  writer =
      std::shared_ptr<Writer<PFPacket>>(new PFWriter<PFPacket>(std::move(transport_), parser, node_->get_logger()));
  return std::make_unique<Pipeline>(writer, reader_, std::bind(&PFInterface::connection_failure_cb, this), net_mtx,
                                    net_cv, net_fail);
}
