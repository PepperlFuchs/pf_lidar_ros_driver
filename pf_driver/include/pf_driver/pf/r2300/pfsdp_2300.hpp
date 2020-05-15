#pragma once

#include "pf_driver/pf/pfsdp_protocol.hpp"

class PFSDP_2300 : public PFSDPBase
{

public:
  PFSDP_2300(const utility::string_t &host) : PFSDPBase(host)
  {
  }

  virtual std::vector<int> get_layers_enabled()
  {
    std::string layers = get_parameter_str("layer_enable");
    std::vector<std::string> vals = split(layers);
    std::vector<int> enabled_layers(vals.size(), 0);
    for (int i = 0; i < vals.size(); i++)
    {
      if (vals[i].compare("on") == 0)
      {
        enabled_layers[i] = 1;
      }
    }
    return enabled_layers;
  }

  std::int32_t get_start_angle(std::string handle)
  {
    std::string angle_str = get_scanoutput_config(std::string("start_angle"), handle);
    std::int32_t start_angle = stof(angle_str);
    return start_angle / 10000;
  }

  virtual std::pair<float, float> get_angle_min_max(std::string handle)
  {
    float measure_start_angle = get_parameter_int(std::string("measure_start_angle")) / 10000 * M_PI / 180.0;
    float measure_stop_angle = get_parameter_int(std::string("measure_stop_angle")) / 10000 * M_PI / 180.0;
    float start_angle = get_start_angle(handle) * M_PI / 180.0;
    std::string scan_direction = get_parameter_str(std::string("scan_direction"));

    float min = (measure_start_angle > start_angle) ? measure_start_angle : start_angle;
    float max = measure_stop_angle;
    return std::pair<float, float>(min, max);
  }

  virtual std::string get_start_angle_str()
  {
      return std::string("start_angle");
  }

  //TODO
  //have a list of supported commands for respective device
  //if command not found in list, send error message that command not supported
};