#pragma once

#include "pf_driver/pf/pfsdp_protocol.hpp"

class PFSDP_2300 : public PFSDPBase
{

public:
  PFSDP_2300(const std::string &host) : PFSDPBase(host)
  {
  }

  virtual std::string get_product()
  {
    return get_parameter_str("part");
  }

  virtual ScanParameters get_scan_parameters(int start_angle)
  {
    auto resp = get_parameter("angular_fov", "radial_range_min", "radial_range_max", "measure_start_angle", "measure_stop_angle");
    params.angular_fov = to_float(resp["angular_fov"]) * M_PI / 180.0;
    params.radial_range_max = to_float(resp["radial_range_max"]);
    params.radial_range_min = to_float(resp["radial_range_min"]);

    auto start_stop = get_angle_start_stop(start_angle);
    params.angle_min = start_stop.first;
    params.angle_max = start_stop.second;
    get_layers_enabled(params.layers_enabled);
    return params;
  }

  virtual void handle_reconfig(pf_driver::PFDriverConfig &config, uint32_t level)
  {
    if(level == 1)
    {
      set_parameter({ KV("ip_mode", config.ip_mode) });
    } else if(level == 2)
    {
      set_parameter({ KV("ip_address", config.ip_address) });
    } else if(level == 3)
    {
      set_parameter({ KV("subnet_mask", config.subnet_mask) });
    } else if(level == 4)
    {
      set_parameter({ KV("gateway", config.gateway) });
    } else if(level == 5)
    {
      set_parameter({ KV("user_tag", config.user_tag) });
    } else if(level == 6)
    {
      set_parameter({ KV("layer_enable", config.layer_enable) });
    } else if(level == 7)
    {
      set_parameter({ KV("scan_frequency", config.scan_frequency) });
    } else if(level == 8)
    {
      set_parameter({ KV("scan_direction", config.scan_direction) });
    } else if(level == 9)
    {
      set_parameter({ KV("measure_start_angle", config.measure_start_angle) });
    } else if(level == 10)
    {
      set_parameter({ KV("measure_stop_angle", config.measure_stop_angle) });
    } else if(level == 11)
    {
      set_parameter({ KV("locator_indication", config.locator_indication) });
    } else if(level == 12)
    {
      set_parameter({ KV("pilot_laser", config.pilot_laser) });
    } else if(level == 13)
    {
      set_parameter({ KV("pilot_start_angle", config.pilot_start_angle) });
    } else if(level == 14)
    {
      set_parameter({ KV("pilot_stop_angle", config.pilot_stop_angle) });
    } else if(level == 15)
    {
      set_parameter({ KV("operating_mode", config.operating_mode) });
    } else if(level == 16)
    {
      set_parameter({ KV("address", config.address) });
    } else if(level == 17)
    {
      set_parameter({ KV("port", config.port) });
    } else if(level == 18)
    {
      set_parameter({ KV("packet_type", config.packet_type) });
    } else if(level == 19)
    {
      set_parameter({ KV("packet_crc", config.packet_crc) });
    } else if(level == 20)
    {
      set_parameter({ KV("watchdog", config.watchdog) });
    } else if(level == 21)
    {
      set_parameter({ KV("watchdogtimeout", config.watchdogtimeout) });
    } else if(level == 22)
    {
      set_parameter({ KV("start_angle", config.start_angle) });
    } else if(level == 23)
    {
      set_parameter({ KV("max_num_points_scan", config.max_num_points_scan) });
    } else if(level == 24)
    {
      set_parameter({ KV("skip_scans", config.skip_scans) });
    }
  }

private:
  void get_layers_enabled(uint16_t &enabled)
  {
    std::string layers = get_parameter_str("layer_enable");
    std::vector<std::string> vals = split(layers);
    std::vector<bool> enabled_layers(vals.size(), false);
    for (int i = 0; i < vals.size(); i++)
    {
      if (vals[i].compare("on") == 0)
      {
        enabled += pow(2, i);
      }
    }
  }

  virtual std::pair<float, float> get_angle_start_stop(int start_angle)
  {
    float measure_start_angle = get_parameter_float("measure_start_angle") / 10000.0 * M_PI / 180.0;
    float measure_stop_angle = get_parameter_float("measure_stop_angle") / 10000.0 * M_PI / 180.0;
    start_angle = start_angle * M_PI / 180.0;

    // float min = (measure_start_angle > start_angle) ? measure_start_angle : start_angle;
    // float max = measure_stop_angle;
    return std::pair<float, float>(measure_start_angle, measure_stop_angle);
  }

  virtual std::string get_start_angle_str()
  {
      return std::string("start_angle");
  }
};