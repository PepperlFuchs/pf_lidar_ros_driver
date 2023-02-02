#include <memory>
#include <string>
#include <utility>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node.hpp>

#include "pf_driver/pf/pf_interface.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("pf_driver");

  node->declare_parameter<std::string>("device");
  std::string device = node->get_parameter("device").get_parameter_value().get<std::string>();

  bool init_valid = true;
  std::shared_ptr<HandleInfo> info = std::make_shared<HandleInfo>();

  // ParameterDescriptor can be used to define the dynamic parameters which can allow
  // description and ranges
  // rcl_interfaces::msg::ParameterDescriptor transport_str_desc;
  // transport_str_desc.name = "transport";
  // transport_str_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;

  rcl_interfaces::msg::ParameterDescriptor descriptorScanTopic;
  descriptorScanTopic.name = "Scan topic";
  descriptorScanTopic.description = "Topic on which the LaserScan messages will be published";
  descriptorScanTopic.read_only = true;
  node->declare_parameter<std::string>("scan_topic", "/scan", descriptorScanTopic);

  rcl_interfaces::msg::ParameterDescriptor descriptorFrameId;
  descriptorFrameId.name = "Scan frame ID";
  descriptorFrameId.description = "Frame ID in which the LaserScan messages will be published";
  descriptorFrameId.read_only = true;
  node->declare_parameter<std::string>("frame_id", "scanner", descriptorScanTopic);

  // declare parameters with device-specific namespace
  std::map<std::string, std::string> device_params = { { "transport", "udp" },
                                                       { "scanner_ip", "" },
                                                       { "port", "" },
                                                       { "start_angle", "" },
                                                       { "max_num_points_scan", "" },
                                                       { "packet_type", "" },
                                                       { "watchdogtimeout", "" },
                                                       { "watchdog", "" },
                                                       { "num_layers", "" },
                                                       { "apply_correction", "" } };
  node->declare_parameters(device, device_params);

  std::string transport_str = node->get_parameter("transport").get_parameter_value().get<std::string>();
  info->handle_type = transport_str == "udp" ? HandleInfo::HANDLE_TYPE_UDP : HandleInfo::HANDLE_TYPE_TCP;

  info->hostname = node->get_parameter("scanner_ip").get_parameter_value().get<std::string>();
  info->hostname = node->get_parameter("port").get_parameter_value().get<std::string>();

  std::shared_ptr<ScanConfig> config = std::make_shared<ScanConfig>();
  config->start_angle = node->get_parameter("start_angle").get_parameter_value().get<int>();
  config->max_num_points_scan = node->get_parameter("max_num_points_scan").get_parameter_value().get<int>();
  config->packet_type = node->get_parameter("packet_type").get_parameter_value().get<std::string>();
  config->watchdogtimeout = node->get_parameter("watchdogtimeout").get_parameter_value().get<int>();
  config->watchdog = node->get_parameter("watchdog").get_parameter_value().get<bool>();

  int num_layers = node->get_parameter("num_layers").get_parameter_value().get<int>();
  std::string topic = node->get_parameter("scan_topic").get_parameter_value().get<std::string>();
  std::string frame_id = node->get_parameter("frame_id").get_parameter_value().get<std::string>();

  std::shared_ptr<ScanParameters> params = std::make_shared<ScanParameters>();
  params->apply_correction = node->get_parameter("apply_correction").get_parameter_value().get<bool>();

  PFInterface pf_interface(node);

  std::shared_ptr<std::mutex> net_mtx_ = std::make_shared<std::mutex>();
  std::shared_ptr<std::condition_variable> net_cv_ = std::make_shared<std::condition_variable>();
  bool net_fail = false;
  bool retrying = false;

  while (rclcpp::ok())
  {
    net_fail = false;
    if (!pf_interface.init(info, config, params, topic, frame_id, num_layers))
    {
      RCLCPP_ERROR(node->get_logger(), "Unable to initialize device");
      if (retrying)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        continue;
      }
      return -1;
    }
    if (!pf_interface.start_transmission(net_mtx_, net_cv_, net_fail))
    {
      RCLCPP_ERROR(node->get_logger(), "Unable to start scan");
      return -1;
    }
    retrying = true;
    // wait for condition variable
    std::unique_lock<std::mutex> net_lock(*net_mtx_);
    net_cv_->wait(net_lock, [&net_fail] { return net_fail; });
    RCLCPP_ERROR(node->get_logger(), "Network failure");
    pf_interface.terminate();
  }

  rclcpp::shutdown();
  pf_interface.stop_transmission();
  return 0;
}
