#include <memory>
#include <string>
#include <utility>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

#include "pf_driver/pf/pf_interface.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("pf_driver");

  /*std::string device;
  nh.getParam("device", device);

  ros::NodeHandle dev_nh("~/" + device);*/

  PFInterface pf_interface(node);

  std::shared_ptr<ScanConfig> config = std::make_shared<ScanConfig>();
  std::shared_ptr<ScanParameters> params = std::make_shared<ScanParameters>();

  while (rclcpp::ok())
  {
    RCLCPP_ERROR(node->get_logger(), "Unable to initialize device");
    return -1;
  }
  if (!pf_interface.start_transmission())
  {
    RCLCPP_ERROR(node->get_logger(), "Unable to start scan");
    return -1;
  }
  rclcpp::spin(node);
  pf_interface.stop_transmission();
  return 0;
}
