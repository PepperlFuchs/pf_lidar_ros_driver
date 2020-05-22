#pragma once

#include "pf_driver/pf/pfsdp_protocol.hpp"

class PFSDP_2300 : public PFSDPBase
{

public:
  PFSDP_2300(const utility::string_t &host) : PFSDPBase(host)
  {
  }

  virtual std::string get_product()
  {
    return get_parameter_str("part");
  }

  virtual ScanParameters get_scan_parameters(int start_angle)
  {
    auto resp = get_parameter("angular_fov", "radial_range_min", "radial_range_max", "measure_start_angle", "measure_stop_angle");

    ScanParameters params;
    params.angular_fov = to_float(resp["angular_fov"]) * M_PI / 180.0;
    params.radial_range_max = to_float(resp["radial_range_max"]);
    params.radial_range_min = to_float(resp["radial_range_min"]);

    auto start_stop = get_angle_start_stop(start_angle);
    params.angle_min = start_stop.first;
    params.angle_max = start_stop.second;
    get_layers_enabled(params.layers_enabled);
    return params;
  }

private:
  void get_layers_enabled(bool *enabled)
  {
    std::string layers = get_parameter_str("layer_enable");
    std::vector<std::string> vals = split(layers);
    std::vector<bool> enabled_layers(vals.size(), false);
    for (int i = 0; i < vals.size(); i++)
    {
      if (vals[i].compare("on") == 0)
      {
        enabled[i] = true;
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