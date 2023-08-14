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

  std::string device, transport_str, scanner_ip, port, topic, frame_id, packet_type;
  int scan_frequency, samples_per_scan, start_angle, max_num_points_scan, watchdogtimeout, num_layers;
  num_layers = 0;
  bool watchdog, apply_correction = 0;
  node->declare_parameter("device", device);
  node->get_parameter("device", device);
  RCLCPP_INFO(node->get_logger(), "device name: %s", device.c_str());

  node->declare_parameter("transport", transport_str);
  node->get_parameter("transport", transport_str);
  RCLCPP_INFO(node->get_logger(), "transport_str: %s", transport_str.c_str());

  node->declare_parameter("scanner_ip", scanner_ip);
  node->get_parameter("scanner_ip", scanner_ip);
  RCLCPP_INFO(node->get_logger(), "scanner_ip: %s", scanner_ip.c_str());

  node->declare_parameter("port", port);
  node->get_parameter("port", port);
  RCLCPP_INFO(node->get_logger(), "port: %s", port.c_str());

  node->declare_parameter("scan_frequency", scan_frequency);
  node->get_parameter("scan_frequency", scan_frequency);
  RCLCPP_INFO(node->get_logger(), "scan_frequency: %d", scan_frequency);

  node->declare_parameter("start_angle", start_angle);
  node->get_parameter("start_angle", start_angle);
  RCLCPP_INFO(node->get_logger(), "start_angle: %d", start_angle);

  node->declare_parameter("max_num_points_scan", max_num_points_scan);
  node->get_parameter("max_num_points_scan", max_num_points_scan);
  RCLCPP_INFO(node->get_logger(), "max_num_points_scan: %d", max_num_points_scan);

  node->declare_parameter("watchdogtimeout", watchdogtimeout);
  node->get_parameter("watchdogtimeout", watchdogtimeout);
  RCLCPP_INFO(node->get_logger(), "watchdogtimeout: %d", watchdogtimeout);

  node->declare_parameter("watchdog", watchdog);
  node->get_parameter("watchdog", watchdog);
  RCLCPP_INFO(node->get_logger(), "watchdog: %d", watchdog);

  node->declare_parameter("num_layers", num_layers);
  node->get_parameter("num_layers", num_layers);
  RCLCPP_INFO(node->get_logger(), "num_layers: %d", num_layers);

  node->declare_parameter("scan_topic", topic);
  node->get_parameter("scan_topic", topic);
  RCLCPP_INFO(node->get_logger(), "topic: %s", topic.c_str());

  node->declare_parameter("frame_id", frame_id);
  node->get_parameter("frame_id", frame_id);
  RCLCPP_INFO(node->get_logger(), "frame_id: %s", frame_id.c_str());

  node->declare_parameter("packet_type", packet_type);
  node->get_parameter("packet_type", packet_type);
  RCLCPP_INFO(node->get_logger(), "packet_type: %s", packet_type.c_str());

  node->declare_parameter("apply_correction", apply_correction);
  node->get_parameter("apply_correction", apply_correction);
  RCLCPP_INFO(node->get_logger(), "apply_correction: %d", apply_correction);

  std::shared_ptr<HandleInfo> info = std::make_shared<HandleInfo>();

  info->handle_type = transport_str == "udp" ? HandleInfo::HANDLE_TYPE_UDP : HandleInfo::HANDLE_TYPE_TCP;

  info->hostname = node->get_parameter("scanner_ip").get_parameter_value().get<std::string>();
  info->port = node->get_parameter("port").get_parameter_value().get<std::string>();

  std::shared_ptr<ScanConfig> config = std::make_shared<ScanConfig>();
  config->start_angle = node->get_parameter("start_angle").get_parameter_value().get<int>();
  config->max_num_points_scan = node->get_parameter("max_num_points_scan").get_parameter_value().get<int>();
  config->packet_type = node->get_parameter("packet_type").get_parameter_value().get<std::string>();
  config->watchdogtimeout = node->get_parameter("watchdogtimeout").get_parameter_value().get<int>();
  config->watchdog = node->get_parameter("watchdog").get_parameter_value().get<bool>();
  // config->scan_frequency = scan_frequency;
  // config->samples_per_scan = samples_per_scan;
  RCLCPP_INFO(node->get_logger(), "start_angle: %d", config->start_angle);

  std::shared_ptr<ScanParameters> params = std::make_shared<ScanParameters>();
  params->apply_correction = node->get_parameter("apply_correction").get_parameter_value().get<bool>();

  PFInterface pf_interface(node);

  std::shared_ptr<std::mutex> net_mtx_ = std::make_shared<std::mutex>();
  std::shared_ptr<std::condition_variable> net_cv_ = std::make_shared<std::condition_variable>();
  bool net_fail = false;
  bool retrying = false;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread t([&executor] { executor.spin(); });

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
  t.join();
  return 0;
}
