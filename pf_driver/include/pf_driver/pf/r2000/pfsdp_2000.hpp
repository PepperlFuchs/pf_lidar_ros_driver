#pragma once

#include "pf_driver/pf/pfsdp_protocol.hpp"

class PFSDP_2000 : public PFSDPBase
{
public:
  PFSDP_2000(const std::string& host) : PFSDPBase(host)
  {
  }

  virtual std::string get_product()
  {
    return get_parameter_str("product");
  }

  virtual ScanParameters get_scan_parameters(int start_angle)
  {
    auto resp = get_parameter("angular_fov", "radial_range_min", "radial_range_max", "scan_frequency");
    params.angular_fov = to_float(resp["angular_fov"]) * M_PI / 180.0;
    params.radial_range_max = to_float(resp["radial_range_max"]);
    params.radial_range_min = to_float(resp["radial_range_min"]);

    params.angle_min = start_angle / 10000.0f * M_PI / 180.0;
    params.angle_max = params.angle_min + params.angular_fov;
    params.scan_freq = to_float(resp["scan_frequency"]);
    return params;
  }

  virtual void handle_reconfig(pf_driver::PFDriverR2000Config& config, uint32_t level)
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
      set_parameter({ KV("scan_frequency", config.scan_frequency) });
    }
    else if (level == 6)
    {
      set_parameter({ KV("scan_direction", config.scan_direction) });
    }
    else if (level == 7)
    {
      set_parameter({ KV("samples_per_scan", config.samples_per_scan) });
    }
    else if (level == 8)
    {
      set_parameter({ KV("hmi_display_mode", config.hmi_display_mode) });
    }
    else if (level == 9)
    {
      set_parameter({ KV("hmi_language", config.hmi_language) });
    }
    else if (level == 10)
    {
      set_parameter({ KV("hmi_button_lock", config.hmi_button_lock) });
    }
    else if (level == 11)
    {
      set_parameter({ KV("hmi_parameter_lock", config.hmi_parameter_lock) });
    }
    else if (level == 12)
    {
      set_parameter({ KV("hmi_static_text_1", config.hmi_static_text_1) });
    }
    else if (level == 13)
    {
      set_parameter({ KV("hmi_static_text_2", config.hmi_static_text_2) });
    }
    else if (level == 14)
    {
      set_parameter({ KV("locator_indication", config.locator_indication) });
    }
    else if (level == 15)
    {
      set_parameter({ KV("operating_mode", config.operating_mode) });
    }
    else if (level == 25)
    {
      set_parameter({ KV("user_tag", config.user_tag) });
    }
    else if (level == 26)
    {
      set_parameter({ KV("user_notes", config.user_notes) });
    }
  }
};