#include "pf_driver/pf/r2300/pfsdp_2300.h"

#include "pf_driver/pf/parser_utils.h"

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
  param_server_R2300_ = std::make_unique<dynamic_reconfigure::Server<pf_driver::PFDriverR2300Config>>();
  param_server_R2300_->setCallback(
      boost::bind(&PFSDP_2300::reconfig_callback, this, boost::placeholders::_1, boost::placeholders::_2));
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
      enabled += pow(2, i);
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

void PFSDP_2300::reconfig_callback(pf_driver::PFDriverR2300Config& config, uint32_t level)
{
  if (level == 1)
  {
    set_parameter({ KV("ip_mode", config.ip_mode) });
  }
  else if (level == 2)
  {
    set_parameter({ KV("ip_address", config.ip_address) });
  }
  else if (level == 3)
  {
    set_parameter({ KV("subnet_mask", config.subnet_mask) });
  }
  else if (level == 4)
  {
    set_parameter({ KV("gateway", config.gateway) });
  }
  else if (level == 5)
  {
    set_parameter({ KV("user_tag", config.user_tag) });
  }
  else if (level == 6)
  {
    set_parameter({ KV("layer_enable", config.layer_enable) });
  }
  else if (level == 7)
  {
    set_parameter({ KV("scan_frequency", config.scan_frequency) });
  }
  else if (level == 8)
  {
    set_parameter({ KV("scan_direction", config.scan_direction) });
  }
  else if (level == 9)
  {
    set_parameter({ KV("measure_start_angle", config.measure_start_angle) });
  }
  else if (level == 10)
  {
    set_parameter({ KV("measure_stop_angle", config.measure_stop_angle) });
  }
  else if (level == 11)
  {
    set_parameter({ KV("locator_indication", config.locator_indication) });
  }
  else if (level == 12)
  {
    set_parameter({ KV("pilot_laser", config.pilot_laser) });
  }
  else if (level == 13)
  {
    set_parameter({ KV("pilot_start_angle", config.pilot_start_angle) });
  }
  else if (level == 14)
  {
    set_parameter({ KV("pilot_stop_angle", config.pilot_stop_angle) });
  }
  else if (level == 15)
  {
    set_parameter({ KV("operating_mode", config.operating_mode) });
  }
  else if (level == 18)
  {
    config_->packet_type = config.packet_type;
  }
  else if (level == 19)
  {
    // currently always none for R2300
    // config_->packet_crc = config.packet_crc;
  }
  else if (level == 20)
  {
    config_->watchdog = (config.watchdog == "on") ? true : false;
  }
  else if (level == 21)
  {
    config_->watchdogtimeout = config.watchdogtimeout;
  }
  else if (level == 22)
  {
    config_->start_angle = config.start_angle;
  }
  else if (level == 23)
  {
    config_->max_num_points_scan = config.max_num_points_scan;
  }
  else if (level == 24)
  {
    config_->skip_scans = config.skip_scans;
  }
  update_scanoutput_config();
}
