#pragma once

#include "pf_driver/pf/pfsdp_base.h"

class PFSDP_2300 : public PFSDPBase
{
public:
  PFSDP_2300(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
             std::shared_ptr<ScanParameters> params);

  virtual std::string get_product();

  virtual std::string get_part();

  virtual void get_scan_parameters();

  void setup_param_server();

private:
  std::shared_ptr<rclcpp::Node> node_;

  void get_layers_enabled(uint16_t& enabled, uint16_t& highest);

  virtual std::pair<float, float> get_angle_start_stop(int start_angle);

  virtual std::string get_start_angle_str();

  virtual bool reconfig_callback_impl(const std::vector<rclcpp::Parameter>& parameters) override;

  virtual void declare_specific_parameters() override;
};
