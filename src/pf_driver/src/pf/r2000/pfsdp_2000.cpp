#include "pf_driver/pf/r2000/pfsdp_2000.h"

#include "pf_driver/pf/parser_utils.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PFSDP_2000::PFSDP_2000(std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
                       std::shared_ptr<ScanParameters> params)
  : PFSDPBase(info, config, params)
{
}

std::string PFSDP_2000::get_product()
{
  return get_parameter_str("product");
}

void PFSDP_2000::get_scan_parameters()
{
  auto resp = get_parameter("angular_fov", "radial_range_min", "radial_range_max", "scan_frequency");
  params_->angular_fov = parser_utils::to_float(resp["angular_fov"]) * M_PI / 180.0;
  params_->radial_range_max = parser_utils::to_float(resp["radial_range_max"]);
  params_->radial_range_min = parser_utils::to_float(resp["radial_range_min"]);

  params_->angle_min = config_->start_angle / 10000.0f * M_PI / 180.0;
  params_->angle_max = params_->angle_min + params_->angular_fov;
  params_->scan_freq = parser_utils::to_float(resp["scan_frequency"]);
}

void PFSDP_2000::setup_param_server()
{
}
