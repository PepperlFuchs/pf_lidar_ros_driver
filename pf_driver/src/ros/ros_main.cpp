#include <memory>
#include <string>
#include <utility>
#include <ros/ros.h>

#include "pf_driver/communication.h"
#include "pf_driver/pf/pf_interface.h"

int main(int argc, char* argv[])
{
  // TODO(ipa-hsd): use argparse
  if (argc < 3)
  {
    ROS_ERROR("Please provide the IP address and the port");
    return -1;
  }
  std::string transport_str = argv[1];
  std::string IP = argv[2];
  std::string port = argv[3];
  std::string device = argv[4];

  ros::init(argc, argv, "pf_driver");
  ros::NodeHandle nh;

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
  // PFInterface pf_interface(transport, IP);
  if (!pf_interface.init())
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
