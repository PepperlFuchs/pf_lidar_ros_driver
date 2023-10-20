#include <iostream>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>

#include "pf_driver/pf/pfsdp_base.h"
#include "pf_driver/pf/parser_utils.h"

PFSDPBase::PFSDPBase(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<HandleInfo> info,
                     std::shared_ptr<ScanConfig> config, std::shared_ptr<ScanParameters> params)
  : http_interface(new HTTPInterface(info->hostname, "cmd")), node_(node), info_(info), config_(config), params_(params)
{
}

const std::map<std::string, std::string> PFSDPBase::get_request(const std::string& command,
                                                                const std::vector<std::string>& json_keys,
                                                                const std::initializer_list<param_type>& query)
{
  return get_request(command, json_keys, param_map_type(query.begin(), query.end()));
}

const std::map<std::string, std::string> PFSDPBase::get_request(const std::string& command,
                                                                const std::vector<std::string>& json_keys,
                                                                const param_map_type& query)
{
  const std::string err_code = "error_code";
  const std::string err_text = "error_text";
  const std::string err_http = "error_http";
  std::vector<std::string> keys = { err_code, err_text };
  keys.insert(keys.end(), json_keys.begin(), json_keys.end());
  std::map<std::string, std::string> json_resp = http_interface->get(keys, command, query);

  if (!check_error(json_resp, err_code, err_text, err_http))
  {
    return std::map<std::string, std::string>();
  }

  return json_resp;
}

bool PFSDPBase::get_request_bool(const std::string& command, const std::vector<std::string>& json_keys,
                                 const std::initializer_list<param_type>& query)
{
  std::map<std::string, std::string> resp = get_request(command, json_keys, query);
  if (resp.empty())
  {
    return false;
  }
  return true;
}

bool PFSDPBase::is_connection_failure(const std::string& http_error)
{
  std::string error_1 = "Failed to connect to ";
  std::string error_2 = "No route to host";

  if (http_error.find(error_1) != std::string::npos && http_error.find(error_2) != std::string::npos)
  {
    return true;
  }
  return false;
}

bool PFSDPBase::check_error(std::map<std::string, std::string>& mp, const std::string& err_code,
                            const std::string& err_text, const std::string& err_http)
{
  const std::string http_error = mp[err_http];
  const std::string code = mp[err_code];
  const std::string text = mp[err_text];

  // remove error related key-value pairs
  mp.erase(err_http);
  mp.erase(err_code);
  mp.erase(err_text);

  // check if HTTP has an error
  if (http_error.compare(std::string("OK")))
  {
    std::cerr << "HTTP ERROR: " << http_error << std::endl;
    if (is_connection_failure(http_error))
    {
      if (handle_connection_failure)
      {
        handle_connection_failure();
      }
    }
    return false;
  }

  // check if 'error_code' and 'error_text' does not exist in the response
  // this happens in case of invalid command
  if (!code.compare("--COULD NOT RETRIEVE VALUE--") || !text.compare("--COULD NOT RETRIEVE VALUE--"))
  {
    std::cout << "Invalid command or parameter requested." << std::endl;
    return false;
  }
  // check for error messages in protocol response
  if (code.compare("0") || text.compare("success"))
  {
    std::cout << "protocol error: " << code << " " << text << std::endl;
    return false;
  }
  return true;
}

void PFSDPBase::set_connection_failure_cb(std::function<void()> callback)
{
  handle_connection_failure = callback;
}

const std::vector<std::string> PFSDPBase::list_parameters()
{
  auto resp = get_request("list_parameters", { "parameters" });
  return parser_utils::split(resp["parameters"]);
}

bool PFSDPBase::reboot_device()
{
  return get_request_bool("reboot_device");
}

void PFSDPBase::factory_reset()
{
  get_request("factory_reset");
}

bool PFSDPBase::release_handle(const std::string& handle)
{
  get_request("release_handle", { "" }, { KV("handle", handle) });
  return true;
}

ProtocolInfo PFSDPBase::get_protocol_info()
{
  ProtocolInfo opi;
  auto resp = get_request("get_protocol_info", { "protocol_name", "version_major", "version_minor", "commands" });
  if (resp.empty())
  {
    opi.isError = true;
    return opi;
  }

  opi.version_major = atoi(resp["version_major"].c_str());
  opi.version_minor = atoi(resp["version_minor"].c_str());
  opi.protocol_name = resp["protocol_name"];
  opi.device_family = get_parameter_int("device_family");

  return opi;
}

int64_t PFSDPBase::get_parameter_int(const std::string& param)
{
  std::map<std::string, std::string> resp = get_parameter(param);
  if (resp.empty())
  {
    return std::numeric_limits<std::int64_t>::quiet_NaN();
  }
  return parser_utils::to_long(resp[param]);
}

float PFSDPBase::get_parameter_float(const std::string& param)
{
  std::map<std::string, std::string> resp = get_parameter(param);
  if (resp.empty())
  {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return parser_utils::to_float(resp[param]);
}

std::string PFSDPBase::get_parameter_str(const std::string& param)
{
  std::map<std::string, std::string> resp = get_parameter(param);
  if (resp.empty())
  {
    return std::string("");
  }
  return resp[param];
}

void PFSDPBase::request_handle_tcp(const std::string& port, const std::string& packet_type)
{
  param_map_type query;
  if (!packet_type.empty())
  {
    query["packet_type"] = packet_type;
  }
  else
  {
    query["packet_type"] = config_->packet_type;
  }
  auto resp = get_request("request_handle_tcp", { "handle", "port" }, query);

  info_->handle = resp["handle"];
  info_->port = resp["port"];

  // TODO: port and pkt_type should be updated in config_
}

void PFSDPBase::request_handle_udp(const std::string& packet_type)
{
  param_map_type query = { KV("address", info_->endpoint), KV("port", info_->port) };
  if (!packet_type.empty())
  {
    query["packet_type"] = packet_type;
  }
  else
  {
    query["packet_type"] = config_->packet_type;
  }
  auto resp = get_request("request_handle_udp", { "handle", "port" }, query);
  info_->handle = resp["handle"];
}

void PFSDPBase::get_scanoutput_config(const std::string& handle)
{
  auto resp =
      get_request("get_scanoutput_config",
                  { "start_angle", "packet_type", "watchdogtimeout", "skip_scans", "watchdog", "max_num_points_scan" },
                  { KV("handle", handle) });
  config_->packet_type = resp["packet_type"];
  config_->start_angle = parser_utils::to_long(resp["start_angle"]);
  config_->watchdogtimeout = parser_utils::to_long(resp["watchdogtimeout"]);
  config_->watchdog = (resp["watchdog"] == "off") ? false : true;
  config_->skip_scans = parser_utils::to_long(resp["skip_scans"]);
  config_->max_num_points_scan = parser_utils::to_long(resp["max_num_points_scan"]);
}

bool PFSDPBase::set_scanoutput_config(const std::string& handle, const ScanConfig& config)
{
  param_map_type query = { KV("handle", handle),
                           KV("start_angle", config.start_angle),
                           KV("packet_type", config.packet_type),
                           KV("max_num_points_scan", config.max_num_points_scan),
                           KV("watchdogtimeout", config.watchdogtimeout),
                           KV("skip_scans", config.skip_scans),
                           KV("watchdog", config.watchdog ? "on" : "off") };
  auto resp = get_request("set_scanoutput_config", { "" }, query);

  // update global config_
  get_scanoutput_config(handle);
  get_scan_parameters();
  return true;
}

bool PFSDPBase::update_scanoutput_config()
{
  param_map_type query = { KV("handle", info_->handle),
                           KV("start_angle", config_->start_angle),
                           KV("packet_type", config_->packet_type),
                           KV("max_num_points_scan", config_->max_num_points_scan),
                           KV("watchdogtimeout", config_->watchdogtimeout),
                           KV("skip_scans", config_->skip_scans),
                           KV("watchdog", config_->watchdog ? "on" : "off") };
  auto resp = get_request("set_scanoutput_config", { "" }, query);

  // recalculate scan params
  get_scan_parameters();
  return true;
}

bool PFSDPBase::start_scanoutput()
{
  get_request("start_scanoutput", { "" }, { { "handle", info_->handle } });
  return true;
}

bool PFSDPBase::stop_scanoutput(const std::string& handle)
{
  return get_request_bool("stop_scanoutput", { "" }, { { "handle", handle } });
}

std::string PFSDPBase::get_scanoutput_config(const std::string& param, const std::string& handle)
{
  auto resp = get_request("get_scanoutput_config", { param }, { KV("handle", handle) });
  return resp[param];
}

bool PFSDPBase::feed_watchdog(const std::string& handle)
{
  return get_request_bool("feed_watchdog", { "" }, { { "handle", handle } });
}

std::string PFSDPBase::get_product()
{
  return std::string("");
}

std::string PFSDPBase::get_part()
{
  return std::string("");
}

void PFSDPBase::get_scan_parameters()
{
}

// handle "dynamic" parameters

rcl_interfaces::msg::SetParametersResult PFSDPBase::reconfig_callback(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = reconfig_callback_impl(parameters);

  if (result.successful)
  {
    update_scanoutput_config();
  }

  return result;
}

bool PFSDPBase::reconfig_callback_impl(const std::vector<rclcpp::Parameter>& parameters)
{
  bool successful = true;

  for (const auto& parameter : parameters)
  {
    if (parameter.get_name() == "ip_mode" || parameter.get_name() == "scan_frequency" ||
        parameter.get_name() == "subnet_mask" || parameter.get_name() == "gateway" ||
        parameter.get_name() == "scan_direction" || parameter.get_name() == "user_tag" ||
        parameter.get_name() == "ip_address" || parameter.get_name() == "subnet_mask" ||
        parameter.get_name() == "gateway")
    {
      return set_parameter({ KV(parameter.get_name(), parameter.value_to_string()) });
    }
    else if (parameter.get_name() == "ip_address")
    {
      info_->hostname = parameter.as_string();
      return set_parameter({ KV(parameter.get_name(), parameter.value_to_string()) });
    }
    else if (parameter.get_name() == "packet_crc")
    {
      return set_parameter({ KV(parameter.get_name(), parameter.as_int()) });
    }
    else if (parameter.get_name() == "port")
    {
      info_->port = parameter.value_to_string();
    }
    else if (parameter.get_name() == "transport")
    {
      // selecting TCP as default if not UDP
      std::string transport_str = parameter.as_string();
      info_->handle_type = transport_str == "udp" ? HandleInfo::HANDLE_TYPE_UDP : HandleInfo::HANDLE_TYPE_TCP;
    }
    else if (parameter.get_name() == "watchdog")
    {
      config_->watchdog = parameter.as_bool();
    }
    else if (parameter.get_name() == "watchdogtimeout")
    {
      config_->watchdogtimeout = parameter.as_int();
    }
    else if (parameter.get_name() == "start_angle")
    {
      config_->start_angle = parameter.as_double();
    }
    else if (parameter.get_name() == "max_num_points_scan")
    {
      config_->max_num_points_scan = parameter.as_int();
    }
    else if (parameter.get_name() == "skip_scans")
    {
      config_->skip_scans = parameter.as_int();
    }
    else if (parameter.get_name() == "locator_indication")
    {
      return set_parameter({ KV(parameter.get_name(), parameter.as_bool() ? "on" : "off") });
    }
  }

  return successful;
}

void PFSDPBase::declare_common_parameters()
{
}

void PFSDPBase::setup_parameters_callback()
{
  parameters_handle_ =
      node_->add_on_set_parameters_callback(std::bind(&PFSDPBase::reconfig_callback, this, std::placeholders::_1));
}
