#include <memory>
#include <string>
#include <utility>
#include <ros/ros.h>

#include "pf_driver/pf/pf_interface.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pf_driver");
  ros::NodeHandle nh("~");

  std::string device;
  nh.getParam("device", device);

  ros::NodeHandle dev_nh("~/" + device);

  std::string transport_str;
  bool init_valid = true;
  std::shared_ptr<HandleInfo> info = std::make_shared<HandleInfo>();
  init_valid &= dev_nh.getParam("transport", transport_str);
  // selecting TCP as default if not UDP
  info->handle_type = transport_str == "udp" ? HandleInfo::HANDLE_TYPE_UDP : HandleInfo::HANDLE_TYPE_TCP;
  init_valid &= dev_nh.getParam("scanner_ip", info->hostname);
  dev_nh.param<std::string>("port", info->port, "0");

  if (!init_valid)
  {
    ROS_ERROR("Please provide the IP address and the port");
    return 1;
  }

  std::shared_ptr<ScanConfig> config = std::make_shared<ScanConfig>();
  // other parameters can also be set in the same way
  int max_num_points_scan = 0;
  int watchdogtimeout = 0;
  bool watchdog;
  int num_layers = 0;
  bool apply_correction;
  dev_nh.getParam("start_angle", config->start_angle);
  dev_nh.getParam("max_num_points_scan", max_num_points_scan);
  dev_nh.getParam("packet_type", config->packet_type);
  dev_nh.getParam("watchdogtimeout", watchdogtimeout);
  dev_nh.getParam("watchdog", watchdog);
  dev_nh.getParam("num_layers", num_layers);
  dev_nh.param<bool>("apply_correction", apply_correction, false);

  config->max_num_points_scan = max_num_points_scan;
  config->watchdogtimeout = watchdogtimeout;
  config->watchdog = watchdog;

  std::string topic, frame_id;
  dev_nh.getParam("scan_topic", topic);
  dev_nh.getParam("frame_id", frame_id);

  // currently ScanParameters is not set through params
  std::shared_ptr<ScanParameters> params = std::make_shared<ScanParameters>();
  params->apply_correction = apply_correction;

  ros::AsyncSpinner spinner(0);
  spinner.start();

  PFInterface pf_interface;

  std::shared_ptr<std::mutex> net_mtx_ = std::make_shared<std::mutex>();
  std::shared_ptr<std::condition_variable> net_cv_ = std::make_shared<std::condition_variable>();
  bool net_fail = false;

  bool retrying = false;

  while (ros::ok())
  {
    net_fail = false;
    if (!pf_interface.init(info, config, params, topic, frame_id, num_layers))
    {
      ROS_ERROR("Unable to initialize device");
      if (retrying)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        continue;
      }
      return -1;
    }
    if (!pf_interface.start_transmission(net_mtx_, net_cv_, net_fail))
    {
      ROS_ERROR("Unable to start scan");
      return -1;
    }
    retrying = true;
    {
      // wait for condition variable
      std::unique_lock<std::mutex> net_lock(*net_mtx_);
      while (ros::ok() &&
             !net_cv_->wait_for(net_lock, std::chrono::milliseconds(1000), [&net_fail] { return net_fail; }))
      {
      };
      if (ros::ok())
      {
        ROS_ERROR("Network failure");
      }
    }
    pf_interface.terminate();
  }

  pf_interface.stop_transmission();
  return 0;
}
