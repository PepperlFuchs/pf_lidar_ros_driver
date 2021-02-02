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

  std::string transport_str, IP, port, device;
  bool init_valid = true;
  init_valid &= nh.getParam("transport", transport_str);
  init_valid &= nh.getParam("scanner_ip", IP);
  init_valid &= nh.getParam("port", port);
  init_valid &= nh.getParam("device", device);

  if (!init_valid)
  {
    ROS_ERROR("Please provide the IP address and the port");
    return 1;
  }

  // other parameters can also be set in the same way
  int max_num_points_scan = 0;
  ScanConfig config;
  nh.getParam("start_angle", config.start_angle);
  nh.getParam("max_num_points_scan", max_num_points_scan);
  config.max_num_points_scan = max_num_points_scan;

  std::unique_ptr<Transport> transport;
  if (transport_str == "udp")
    transport = std::make_unique<UDPTransport>(IP);
  else if (transport_str == "tcp")
    transport = std::make_unique<TCPTransport>(IP);
  else
  {
    ROS_ERROR("Incorrect transport option.");
    return -1;
  }
  PFInterface pf_interface(std::move(transport), device);
  if (!pf_interface.init())
  {
    ROS_ERROR("Unable to initialize device");
    return -1;
  }
  if (!pf_interface.start_transmission(config))
  {
    ROS_ERROR("Unable to start scan");
    return -1;
  }
  ros::spin();
  pf_interface.stop_transmission();
  return 0;
}
