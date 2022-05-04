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

  std::string transport_str, device;
  bool init_valid = true;
  std::shared_ptr<HandleInfo> info = std::make_shared<HandleInfo>();
  init_valid &= nh.getParam("transport", transport_str);
  // selecting TCP as default if not UDP
  info->handle_type = transport_str == "udp" ? HandleInfo::HANDLE_TYPE_UDP : HandleInfo::HANDLE_TYPE_TCP;
  init_valid &= nh.getParam("scanner_ip", info->hostname);
  init_valid &= nh.getParam("port", info->port);
  init_valid &= nh.getParam("device", device);

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
  nh.getParam("start_angle", config->start_angle);
  nh.getParam("max_num_points_scan", max_num_points_scan);
  nh.getParam("packet_type", config->packet_type);
  nh.getParam("watchdogtimeout", watchdogtimeout);
  nh.getParam("watchdogtimeout", watchdog);

  config->max_num_points_scan = max_num_points_scan;
  config->watchdogtimeout = watchdogtimeout;
  config->watchdog = watchdog;

  std::string topic, frame_id;
  nh.getParam("scan_topic", topic);
  nh.getParam("frame_id", frame_id);

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
