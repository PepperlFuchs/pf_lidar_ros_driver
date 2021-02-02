#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "pf_driver/pf/pf_interface.h"
#include "pf_driver/ros/scan_publisher.h"

bool PFInterface::init()
{
  // This is the first time ROS communicates with the device
  auto opi = protocol_interface_->get_protocol_info();
  if (opi.isError)
  {
    ROS_ERROR("Unable to communicate with device. Please check the IP address");
    return false;
  }
  // ROS_INFO("Info: %i %i %s", opi.version_major, opi.version_minor, opi.protocol_name.c_str());

  if (opi.protocol_name != "pfsdp")
    return false;
  if (!handle_version(opi.version_major, opi.version_minor))
    return false;
  setup_param_server();

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

bool PFInterface::handle_version(int major_version, int minor_version)
{
  std::string product_name = "product";
  if (expected_device_ == "R2000")
  {
    protocol_interface_ = std::make_shared<PFSDP_2000>(ip_);
  }
  else if (expected_device_ == "R2300")
  {
    protocol_interface_ = std::make_shared<PFSDP_2300>(ip_);
  }
  product_name = protocol_interface_->get_product();
  if (product_name.find(expected_device_) != std::string::npos)
  {
    ROS_INFO("Device found: %s", product_name.c_str());
    product_ = expected_device_;
    return true;
  }
  ROS_ERROR("Device unsupported");
  return false;
}

void PFInterface::setup_param_server()
{
  if (expected_device_ == "R2000")
  {
    param_server_R2000_ = std::make_unique<dynamic_reconfigure::Server<pf_driver::PFDriverR2000Config>>();
    param_server_R2000_->setCallback(
        boost::bind(&PFInterface::reconfig_callback_r2000, this, boost::placeholders::_1, boost::placeholders::_2));
  }
  else
  {
    param_server_R2300_ = std::make_unique<dynamic_reconfigure::Server<pf_driver::PFDriverR2300Config>>();
    param_server_R2300_->setCallback(
        boost::bind(&PFInterface::reconfig_callback_r2300, this, boost::placeholders::_1, boost::placeholders::_2));
  }
}

bool PFInterface::start_transmission(ScanConfig& config)
{
  if (state_ != PFState::INIT)
    return false;

  if (pipeline_ && pipeline_->is_running())
    return true;

  std::string pkt_type = (expected_device_ == "R2000") ? "C" : "";
  if (transport_type_ == transport_type::tcp)
  {
    info_ = protocol_interface_->request_handle_tcp(port_, pkt_type);
    if (port_.empty())
      port_ = info_.port;
    transport_->set_port(port_);
    transport_->connect();
  }
  else if (transport_type_ == transport_type::udp)
  {
    if (!transport_->connect())
      return false;

    std::string host_ip = transport_->get_host_ip();
    port_ = transport_->get_port();
    info_ = protocol_interface_->request_handle_udp(host_ip, port_, pkt_type);
  }
  if (info_.handle.empty())
    return false;

  config_ = protocol_interface_->get_scanoutput_config(info_.handle);
  config_.start_angle = config.start_angle;
  config_.max_num_points_scan = config.max_num_points_scan;
  params_ = protocol_interface_->get_scan_parameters(config_.start_angle);

  // config_.print();
  // params_.print();

  protocol_interface_->set_scanoutput_config(info_.handle, config_);
  config_ = protocol_interface_->get_scanoutput_config(info_.handle);
  pipeline_ = get_pipeline(config_.packet_type);
  pipeline_->set_scanoutput_config(config_);
  pipeline_->set_scan_params(params_);

  if (!pipeline_->start())
    return false;

  protocol_interface_->start_scanoutput(info_.handle);
  if (config_.watchdog)
    start_watchdog_timer(config_.watchdogtimeout / 1000.0);

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
  protocol_interface_->stop_scanoutput(info_.handle);
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
  std::shared_ptr<Reader<PFPacket>> reader;
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
    reader = std::shared_ptr<Reader<PFPacket>>(new ScanPublisherR2000("/scan", "scanner"));
  }
  else if (product_ == "R2300")
  {
    if (packet_type == "C1")
    {
      parser = std::unique_ptr<Parser<PFPacket>>(new PFR2300_C1_Parser);
    }
    reader = std::shared_ptr<Reader<PFPacket>>(new ScanPublisherR2300("/cloud", "scanner"));
  }
  writer = std::shared_ptr<Writer<PFPacket>>(new PFWriter<PFPacket>(std::move(transport_), parser));
  return std::unique_ptr<Pipeline<PFPacket>>(
      new Pipeline<PFPacket>(writer, reader, std::bind(&PFInterface::on_shutdown, this)));
}

void PFInterface::start_watchdog_timer(float duration)
{
  int feed_time = std::floor(std::min(duration, 60.0f));
  watchdog_timer_ =
      nh_.createTimer(ros::Duration(feed_time), std::bind(&PFInterface::feed_watchdog, this, std::placeholders::_1));
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

void PFInterface::reconfig_callback_r2000(pf_driver::PFDriverR2000Config& config, uint32_t level)
{
  bool watchdog = false;
  uint watchdogtimeout = 0;
  std::string packet_type = "";
  int start_angle = 0;
  uint max_num_points_scan = 0;
  uint skip_scans = 0;
  if (product_ != "R2000")
    return;
  if (state_ != PFState::RUNNING)
    return;
  // Do we want to change the address and port at run-time?
  // if(level == 16){
  //   set_parameter({ KV("address", config.address) });
  // } else if(level == 17)
  // {
  //   set_parameter({ KV("port", config.port) });
  // }
  if (level == 18)
  {
    config_.packet_type = config.packet_type;
  }
  else if (level == 19)
  {
    // this param doesn't exist for R2000
    // set_parameter({ KV("packet_crc", config.packet_crc) });
  }
  else if (level == 20)
  {
    config_.watchdog = (config.watchdog == "on") ? true : false;
  }
  else if (level == 21)
  {
    config_.watchdogtimeout = config.watchdogtimeout;
  }
  else if (level == 22)
  {
    config_.start_angle = config.start_angle;
  }
  else if (level == 23)
  {
    config_.max_num_points_scan = config.max_num_points_scan;
  }
  else if (level == 24)
  {
    config_.skip_scans = config.skip_scans;
  }
  else
  {
    protocol_interface_->handle_reconfig(config, level);
  }
  pipeline_->set_scanoutput_config(config_);
  protocol_interface_->set_scanoutput_config(info_.handle, config_);
  params_ = protocol_interface_->get_scan_parameters(config_.start_angle);
  pipeline_->set_scan_params(params_);
}

void PFInterface::reconfig_callback_r2300(pf_driver::PFDriverR2300Config& config, uint32_t level)
{
  if (product_ != "R2300")
    return;
  if (state_ != PFState::RUNNING)
    return;
  // Do we want to change the address and port at run-time?
  // if(level == 16){
  //   set_parameter({ KV("address", config.address) });
  // } else if(level == 17)
  // {
  //   set_parameter({ KV("port", config.port) });
  // }
  if (level == 18)
  {
    config_.packet_type = config.packet_type;
  }
  else if (level == 19)
  {
    // currently always none for R2300
    // config_.packet_crc = config.packet_crc;
  }
  else if (level == 20)
  {
    config_.watchdog = (config.watchdog == "on") ? true : false;
  }
  else if (level == 21)
  {
    config_.watchdogtimeout = config.watchdogtimeout;
  }
  else if (level == 22)
  {
    config_.start_angle = config.start_angle;
  }
  else if (level == 23)
  {
    config_.max_num_points_scan = config.max_num_points_scan;
  }
  else if (level == 24)
  {
    config_.skip_scans = config.skip_scans;
  }
  else
  {
    protocol_interface_->handle_reconfig(config, level);
  }
  protocol_interface_->set_scanoutput_config(info_.handle, config_);
  config_ = protocol_interface_->get_scanoutput_config(info_.handle);
  pipeline_->set_scanoutput_config(config_);
  params_ = protocol_interface_->get_scan_parameters(config_.start_angle);
  pipeline_->set_scan_params(params_);
}
