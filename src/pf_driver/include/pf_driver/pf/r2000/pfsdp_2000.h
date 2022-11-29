#pragma once

#include <dynamic_reconfigure/server.h>

#include "pf_driver/pf/pfsdp_base.h"
#include "pf_driver/PFDriverR2000Config.h"

class PFSDP_2000 : public PFSDPBase
{
public:
  PFSDP_2000(std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
             std::shared_ptr<ScanParameters> params);

  virtual std::string get_product();

  virtual void get_scan_parameters();

  void setup_param_server();

  virtual void reconfig_callback(pf_driver::PFDriverR2000Config& config, uint32_t level);

private:
  std::unique_ptr<dynamic_reconfigure::Server<pf_driver::PFDriverR2000Config>> param_server_R2000_;
};
