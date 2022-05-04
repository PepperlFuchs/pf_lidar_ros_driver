#include <memory>
#include <string>
#include <utility>
#include <ros/ros.h>

#include "pf_driver/communication.h"
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
  dev_nh.getParam("port", info->port);

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
  dev_nh.getParam("start_angle", config->start_angle);
  dev_nh.getParam("max_num_points_scan", max_num_points_scan);
  dev_nh.getParam("packet_type", config->packet_type);
  dev_nh.getParam("watchdogtimeout", watchdogtimeout);
  dev_nh.getParam("watchdogtimeout", watchdog);

  config->max_num_points_scan = max_num_points_scan;
  config->watchdogtimeout = watchdogtimeout;
  config->watchdog = watchdog;

  std::string topic, frame_id;
  dev_nh.getParam("scan_topic", topic);
  dev_nh.getParam("frame_id", frame_id);

  // currently ScanParameters is not set through params
  std::shared_ptr<ScanParameters> params = std::make_shared<ScanParameters>();

  PFInterface pf_interface;
  if (!pf_interface.init(info, config, params, topic, frame_id))
  {
    ROS_ERROR("Unable to initialize device");
    return -1;
  }
  if (!pf_interface.start_transmission())
  {
    ROS_ERROR("Unable to start scan");
    return -1;
  }
  ros::spin();
  pf_interface.stop_transmission();
  return 0;
}
