#pragma once

#include "pf_driver/pf/pfsdp_protocol.hpp"

class PFSDP_2300 : public PFSDPBase
{
public:
  PFSDP_2300(std::shared_ptr<PFSDPBase> base)
    : PFSDPBase(base)
  {
    config_->packet_type = "C1";

    declare_specific_parameters();
  }

  virtual std::string get_product()
  {
    return get_parameter_str("product");
  }

  virtual void read_scan_parameters()
  {
    auto resp = get_parameter("angular_fov", "radial_range_min", "radial_range_max", "measure_start_angle",
                              "measure_stop_angle", "scan_frequency");
    params_->angular_fov = to_float(resp["angular_fov"]) * M_PI / 180.0;
    params_->radial_range_max = to_float(resp["radial_range_max"]);
    params_->radial_range_min = to_float(resp["radial_range_min"]);

    auto start_stop = get_angle_start_stop(config_->start_angle);
    params_->angle_min = start_stop.first;
    params_->angle_max = start_stop.second;
    get_layers_enabled(params_->layers_enabled, params_->h_enabled_layer);
    params_->scan_freq = to_float(resp["scan_frequency"]);
  }

  void declare_specific_parameters() override
  {
  }

private:
  void get_layers_enabled(uint16_t& enabled, uint16_t& highest)
  {
    enabled = 0;
    std::string layers = get_parameter_str("layer_enable");
    std::vector<std::string> vals = split(layers);
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

  virtual bool reconfig_callback_impl(const std::vector<rclcpp::Parameter>& parameters) override
  {
    bool successful = PFSDPBase::reconfig_callback_impl(parameters);

    for(const auto &parameter : parameters)
    {
      if(parameter.get_name() == "measure_start_angle" ||
         parameter.get_name() == "measure_stop_angle" ||
         parameter.get_name() == "pilot_start_angle" ||
         parameter.get_name() == "pilot_stop_angle" ||
         parameter.get_name() == "layer_enable" ||
         parameter.get_name() == "pilot_laser")
      {
        set_parameter({ KV(parameter.get_name(), parameter.value_to_string()) });
      }
    }

    return successful;
  }
};
