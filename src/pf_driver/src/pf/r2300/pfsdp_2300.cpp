#include <cmath>

#include "pf_driver/pf/r2300/pfsdp_2300.h"
#include "pf_driver/pf/parser_utils.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

PFSDP_2300::PFSDP_2300(std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
                       std::shared_ptr<ScanParameters> params)
  : PFSDPBase(info, config, params)
{
}

std::string PFSDP_2300::get_product()
{
  return get_parameter_str("product");
}

std::string PFSDP_2300::get_part()
{
  return get_parameter_str("product");
}

void PFSDP_2300::get_scan_parameters()
{
  auto resp = get_parameter("angular_fov", "radial_range_min", "radial_range_max", "measure_start_angle",
                            "measure_stop_angle", "scan_frequency");
  params_->angular_fov = parser_utils::to_float(resp["angular_fov"]) * M_PI / 180.0;
  params_->radial_range_max = parser_utils::to_float(resp["radial_range_max"]);
  params_->radial_range_min = parser_utils::to_float(resp["radial_range_min"]);

  auto start_stop = get_angle_start_stop(config_->start_angle);
  params_->angle_min = start_stop.first;
  params_->angle_max = start_stop.second;
  get_layers_enabled(params_->layers_enabled, params_->h_enabled_layer);
  params_->scan_freq = parser_utils::to_float(resp["scan_frequency"]);
}

void PFSDP_2300::setup_param_server()
{
}

void PFSDP_2300::get_layers_enabled(uint16_t& enabled, uint16_t& highest)
{
  enabled = 0;
  std::string layers = get_parameter_str("layer_enable");
  std::vector<std::string> vals = parser_utils::split(layers);
  std::vector<bool> enabled_layers(vals.size(), false);
  for (int i = 0; i < vals.size(); i++)
  {
    if (vals[i].compare("on") == 0)
    {
      enabled += std::pow(2, i);
      highest = i;
    }
  }
}

std::pair<float, float> PFSDP_2300::get_angle_start_stop(int start_angle)
{
  float measure_start_angle = get_parameter_float("measure_start_angle") / 10000.0 * M_PI / 180.0;
  float measure_stop_angle = get_parameter_float("measure_stop_angle") / 10000.0 * M_PI / 180.0;
  start_angle = start_angle * M_PI / 180.0;

  // float min = (measure_start_angle > start_angle) ? measure_start_angle : start_angle;
  // float max = measure_stop_angle;
  return std::pair<float, float>(measure_start_angle, measure_stop_angle);
}

std::string PFSDP_2300::get_start_angle_str()
{
  return std::string("start_angle");
}
