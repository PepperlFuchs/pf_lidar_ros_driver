#pragma once

#include <dynamic_reconfigure/server.h>

#include "pf_driver/pf/pfsdp_base.h"
#include "pf_driver/PFDriverR2300Config.h"

class PFSDP_2300 : public PFSDPBase
{
public:
  PFSDP_2300(std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
             std::shared_ptr<ScanParameters> params);

  virtual std::string get_product();

  virtual std::string get_part();

  virtual void get_scan_parameters();

  void setup_param_server();

private:
  void get_layers_enabled(uint16_t& enabled, uint16_t& highest);

  virtual std::pair<float, float> get_angle_start_stop(int start_angle);

  virtual std::string get_start_angle_str();

  void reconfig_callback(pf_driver::PFDriverR2300Config& config, uint32_t level);

  std::unique_ptr<dynamic_reconfigure::Server<pf_driver::PFDriverR2300Config>> param_server_R2300_;
};
