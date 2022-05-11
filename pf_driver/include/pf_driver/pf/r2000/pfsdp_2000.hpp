#pragma once

#include "pf_driver/pf/pfsdp_protocol.hpp"

class PFSDP_2000 : public PFSDPBase
{
public:
  PFSDP_2000(std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
             std::shared_ptr<ScanParameters> params, std::shared_ptr<std::mutex> config_mutex)
    : PFSDPBase(info, config, params, config_mutex)
  {
  }

  virtual std::string get_product()
  {
    return get_parameter_str("product");
  }

  virtual void get_scan_parameters()
  {
    auto resp = get_parameter("angular_fov", "radial_range_min", "radial_range_max", "scan_frequency");
    params_->angular_fov = to_float(resp["angular_fov"]) * M_PI / 180.0;
    params_->radial_range_max = to_float(resp["radial_range_max"]);
    params_->radial_range_min = to_float(resp["radial_range_min"]);

    params_->angle_min = config_->start_angle / 10000.0f * M_PI / 180.0;
    params_->angle_max = params_->angle_min + params_->angular_fov;
    params_->scan_freq = to_float(resp["scan_frequency"]);
  }

  void setup_param_server()
  {
    param_server_R2000_ = std::make_unique<dynamic_reconfigure::Server<pf_driver::PFDriverR2000Config>>();
    param_server_R2000_->setCallback(
        boost::bind(&PFSDP_2000::reconfig_callback, this, boost::placeholders::_1, boost::placeholders::_2));
  }

  virtual void reconfig_callback(pf_driver::PFDriverR2000Config& config, uint32_t level)
  {
    config_mutex_->lock();

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
    else if (level == 18)
    {
      config_->packet_type = config.packet_type;
    }
    else if (level == 19)
    {
      // this param doesn't exist for R2000
      // set_parameter({ KV("packet_crc", config.packet_crc) });
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

    config_mutex_->unlock();
  }

private:
  std::unique_ptr<dynamic_reconfigure::Server<pf_driver::PFDriverR2000Config>> param_server_R2000_;
};
