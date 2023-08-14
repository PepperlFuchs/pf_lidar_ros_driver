#pragma once

// #include <dynamic_reconfigure/server.h>

#include "pf_driver/pf/pfsdp_base.h"

class PFSDP_2000 : public PFSDPBase
{
public:
  PFSDP_2000(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
             std::shared_ptr<ScanParameters> params);

  virtual std::string get_product();

  virtual void get_scan_parameters();

  virtual void declare_specific_parameters() override;

  virtual bool reconfig_callback_impl(const std::vector<rclcpp::Parameter>& parameters) override;

private:
  std::shared_ptr<rclcpp::Node> node_;
};
