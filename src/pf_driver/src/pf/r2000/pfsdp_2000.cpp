#include "pf_driver/pf/r2000/pfsdp_2000.h"

#include "pf_driver/pf/parser_utils.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PFSDP_2000::PFSDP_2000(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<HandleInfo> info,
                       std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params)
  : PFSDPBase(node, info, config, params)
{
  declare_specific_parameters();
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
  rcl_interfaces::msg::ParameterDescriptor descriptorPacketType;
  descriptorPacketType.name = "Packet type";
  descriptorPacketType.description = "Packet type for scan data output";
  descriptorPacketType.read_only = true;
  node_->declare_parameter<std::string>("packet_type", "C", descriptorPacketType);
}

bool PFSDP_2000::reconfig_callback_impl(const std::vector<rclcpp::Parameter>& parameters)
{
  bool successful = PFSDPBase::reconfig_callback_impl(parameters);

  for (const auto& parameter : parameters)
  {
    if (parameter.get_name() == "samples_per_scan" || parameter.get_name() == "hmi_display_mode" ||
        parameter.get_name() == "hmi_language" || parameter.get_name() == "hmi_button_lock" ||
        parameter.get_name() == "hmi_parameter_lock" || parameter.get_name() == "hmi_static_text_1" ||
        parameter.get_name() == "hmi_static_text_2" || parameter.get_name() == "user_notes")
    {
      set_parameter({ KV(parameter.get_name(), parameter.value_to_string()) });
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
  }

  return successful;
}
