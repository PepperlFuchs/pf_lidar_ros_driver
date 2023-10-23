#include "pf_driver/pf/r2000/pfsdp_2000.h"

#include "pf_driver/pf/parser_utils.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PFSDP_2000::PFSDP_2000(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<HandleInfo> info,
                       std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params)
  : PFSDPBase(node, info, config, params)
{
  node_ = node;
  declare_specific_parameters();

  parameters_handle_ =
      node_->add_on_set_parameters_callback(std::bind(&PFSDP_2000::reconfig_callback, this, std::placeholders::_1));
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

void PFSDP_2000::declare_specific_parameters()
{
  std::string hmi_display_mode, hmi_application_bitmap, operating_mode, hmi_language, hmi_static_text_1,
      hmi_static_text_2, user_notes, filter_type, filter_error_handling, filter_remission_threshold,
      lcm_detection_sensitivity, lcm_sector_enable;

  bool hmi_button_lock, hmi_parameter_lock;

  int samples_per_scan, filter_width, filter_maximum_margin, lcm_detection_period;

  if (!node_->has_parameter("samples_per_scan"))
  {
    node_->declare_parameter("samples_per_scan", samples_per_scan);
  }
  if (!node_->has_parameter("hmi_application_bitmap"))
  {
    node_->declare_parameter("hmi_application_bitmap", hmi_application_bitmap);
  }
  if (!node_->has_parameter("operating_mode"))
  {
    node_->declare_parameter("operating_mode", operating_mode);
  }
  if (!node_->has_parameter("hmi_display_mode"))
  {
    node_->declare_parameter("hmi_display_mode", hmi_display_mode);
  }
  if (!node_->has_parameter("hmi_language"))
  {
    node_->declare_parameter("hmi_language", hmi_language);
  }
  if (!node_->has_parameter("hmi_button_lock"))
  {
    node_->declare_parameter("hmi_button_lock", hmi_button_lock);
  }
  if (!node_->has_parameter("hmi_parameter_lock"))
  {
    node_->declare_parameter("hmi_parameter_lock", hmi_parameter_lock);
  }
  if (!node_->has_parameter("hmi_static_text_1"))
  {
    node_->declare_parameter("hmi_static_text_1", hmi_static_text_1);
  }
  if (!node_->has_parameter("hmi_static_text_2"))
  {
    node_->declare_parameter("hmi_static_text_2", hmi_static_text_2);
  }
  if (!node_->has_parameter("user_notes"))
  {
    node_->declare_parameter("user_notes", user_notes);
  }
  if (!node_->has_parameter("filter_type"))
  {
    node_->declare_parameter("filter_type", filter_type);
  }
  if (!node_->has_parameter("filter_width"))
  {
    node_->declare_parameter("filter_width", filter_width);
  }
  if (!node_->has_parameter("filter_error_handling"))
  {
    node_->declare_parameter("filter_error_handling", filter_error_handling);
  }
  if (!node_->has_parameter("filter_maximum_margin"))
  {
    node_->declare_parameter("filter_maximum_margin", filter_maximum_margin);
  }
  if (!node_->has_parameter("filter_remission_threshold"))
  {
    node_->declare_parameter("filter_remission_threshold", filter_remission_threshold);
  }
  if (!node_->has_parameter("lcm_detection_sensitivity"))
  {
    node_->declare_parameter("lcm_detection_sensitivity", lcm_detection_sensitivity);
  }
  if (!node_->has_parameter("lcm_detection_period"))
  {
    node_->declare_parameter("lcm_detection_period", lcm_detection_period);
  }
  if (!node_->has_parameter("lcm_sector_enable"))
  {
    node_->declare_parameter("lcm_sector_enable", lcm_sector_enable);
  }
}

bool PFSDP_2000::reconfig_callback_impl(const std::vector<rclcpp::Parameter>& parameters)
{
  bool successful = PFSDPBase::reconfig_callback_impl(parameters);

  for (const auto& parameter : parameters)
  {
    if (parameter.get_name() == "hmi_language" || parameter.get_name() == "hmi_static_text_1" ||
        parameter.get_name() == "hmi_static_text_2" || parameter.get_name() == "user_notes" ||
        parameter.get_name() == "operating_mode" || parameter.get_name() == "filter_type" ||
        parameter.get_name() == "filter_error_handling" || parameter.get_name() == "filter_remission_threshold" ||
        parameter.get_name() == "lcm_detection_sensitivity" || parameter.get_name() == "lcm_sector_enable")
    {
      return set_parameter({ KV(parameter.get_name(), parameter.value_to_string()) });
    }
    else if (parameter.get_name() == "packet_type")
    {
      std::string packet_type = parameter.as_string();
      if (packet_type == "A" || packet_type == "B" || packet_type == "C")
      {
        config_->packet_type = packet_type;
      }
      else
      {
        successful = false;
      }
    }
    else if (parameter.get_name() == "samples_per_scan" || parameter.get_name() == "filter_width" ||
             parameter.get_name() == "filter_maximum_margin" || parameter.get_name() == "lcm_detection_period")
    {
      return set_parameter({ KV(parameter.get_name(), parameter.as_int()) });
    }
    else if (parameter.get_name() == "hmi_button_lock" || parameter.get_name() == "hmi_parameter_lock")
    {
      return set_parameter({ KV(parameter.get_name(), parameter.as_bool() ? "on" : "off") });
    }
    else if (parameter.get_name() == "hmi_display_mode")
    {
      return set_parameter({ KV(parameter.get_name(),
                                parameter.value_to_string() == "mode_off" ? "off" : parameter.value_to_string()) });
    }
  }

  return successful;
}
