// Copyright 2019 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PF_DRIVER_PFSDP_PROTOCOL_H
#define PF_DRIVER_PFSDP_PROTOCOL_H

#pragma once

#include <boost/algorithm/string.hpp>
#include "pf_driver/pf/http_helpers.hpp"
#include "pf_driver/PFDriverR2000Config.h"
#include "pf_driver/PFDriverR2300Config.h"

struct ProtocolInfo
{
  bool isError = false;
  std::string protocol_name;          // protocol name, defaults to "pfsdp"
  int version_major;                  // major version of protocol
  int version_minor;                  // minor version of protocol
  std::vector<std::string> commands;  // list of available commands
                                      // Since R2300 may not give correct error reports
                                      // it is safer to keep the list of commands
};

struct HandleInfo
{
  static const int HANDLE_TYPE_TCP = 0;
  static const int HANDLE_TYPE_UDP = 1;

  int handle_type;
  std::string hostname;
  std::string port;
  std::string handle;
};

struct ScanConfig
{
  bool watchdog = false;
  uint watchdogtimeout = 0;
  std::string packet_type = "";
  int start_angle = 0;
  uint max_num_points_scan = 0;
  uint skip_scans = 0;

  // void print()
  // {
  //   std::cout << "Scan output config:\n"
  //             << "watchdogtimeout: " << watchdogtimeout << "\n"
  //             << "packet_type: " << packet_type << "\n"
  //             << "start_angle: " << start_angle << "\n"
  //             << "max_num_points_scan:" << max_num_points_scan << "\n"
  //             << "skip_scan: " << skip_scans << std::endl;
  // }
};

#pragma pack(push, sp, 1)
struct ScanParameters
{
  double angular_fov = 0.0;
  double radial_range_min = 0.0;
  double radial_range_max = 0.0;
  double angle_min = 0.0;
  double angle_max = 0.0;
  uint16_t layers_enabled = 0;
  double scan_freq = 0.0;  // needed to calculate scan resolution in R2300

  // void print()
  // {
  //   std::cout << "Scan parameters:\n"
  //             << "angular_fov: " << angular_fov << "\n"
  //             << "radial_range_min: " << radial_range_min << "\n"
  //             << "radial_range_max: " << radial_range_max << "\n"
  //             << "angle_min: " << angle_min << "\n"
  //             << "angle_max: " << angle_max << "\n"
  //             << "layers enabled: ";
  //   for(auto &layer : layers_enabled)
  //     std::cout << layer << " ";
  //   std::cout << std::endl;
  // }
};
#pragma pack(pop, sp)

class KV : public std::pair<std::string, std::string>
{
  template <typename T>
  static std::string make_list(bool first, const T& t)
  {
    std::stringstream s;
    if (!first)
      s << ";";
    s << t;
    return s.str();
  }

  template <typename T, typename... Ts>
  static std::string make_list(bool first, const T& t, Ts&&... list)
  {
    std::stringstream s;
    s << make_list(first, t);
    s << make_list(false, list...);
    return s.str();
  }

  static std::string make_list(bool first, std::vector<std::string> list)
  {
    std::stringstream s;
    std::copy(list.begin(), list.end() - 1, std::ostream_iterator<std::string>(s, ";"));
    s << list.back();

    return s.str();
  }

public:
  template <typename... Ts>
  KV(const std::string& k, const Ts&... list) : std::pair<std::string, std::string>(k, make_list(true, list...))
  {
  }
};

inline const std::vector<std::string> split(std::string str, const char delim = ';')
{
  std::vector<std::string> results;
  boost::split(results, str, [delim](char c) { return c == delim; });
  return results;
}

inline std::int64_t to_long(const std::string& s)
{
  std::int64_t int_val = 0;
  try
  {
    int_val = std::stoll(s);
  }
  catch (std::exception& e)
  {
    std::cerr << "conversion of data from string failed: " << s << std::endl;
    return std::numeric_limits<std::int64_t>::quiet_NaN();
  }
  return int_val;
}

inline uint16_t to_uint16(const std::string& s)
{
  uint16_t int_val = 0;
  try
  {
    int_val = static_cast<uint16_t>(stoi(s));
  }
  catch (std::exception& e)
  {
    std::cerr << "conversion of data from string failed: " << s << std::endl;
    return std::numeric_limits<uint16_t>::quiet_NaN();
  }
  return int_val;
}

inline float to_float(const std::string& s)
{
  float float_val = 0;
  try
  {
    float_val = std::stof(s);
  }
  catch (std::exception& e)
  {
    std::cerr << "conversion of data from string failed " << s << std::endl;
    return std::numeric_limits<float>::quiet_NaN();
  }
  return float_val;
}

class PFSDPBase
{
private:
  using HTTPInterfacePtr = std::unique_ptr<HTTPInterface>;
  HTTPInterfacePtr http_interface;
  std::string hostname;

  const std::map<std::string, std::string> get_request(const std::string command, std::vector<std::string> json_keys,
                                                       const std::initializer_list<param_type> query)
  {
    return get_request(command, json_keys, param_map_type(query.begin(), query.end()));
  }
  const std::map<std::string, std::string> get_request(const std::string command,
                                                       std::vector<std::string> json_keys = std::vector<std::string>(),
                                                       const param_map_type query = param_map_type())
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

  bool get_request_bool(const std::string command, std::vector<std::string> json_keys = std::vector<std::string>(),
                        std::initializer_list<param_type> query = std::initializer_list<param_type>())
  {
    std::map<std::string, std::string> resp = get_request(command, json_keys, query);
    if (resp.empty())
    {
      return false;
    }
    return true;
  }

  bool check_error(std::map<std::string, std::string>& mp, const std::string& err_code, const std::string& err_text,
                   const std::string& err_http)
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

protected:
  ScanConfig config;
  ScanParameters params;

public:
  PFSDPBase(const std::string& host) : hostname(host), http_interface(new HTTPInterface(host, "cmd"))
  {
  }

  const std::vector<std::string> list_parameters()
  {
    auto resp = get_request("list_parameters", { "parameters" });
    return split(resp["parameters"]);
  }

  bool reboot_device()
  {
    return get_request_bool("reboot_device");
  }

  void factory_reset()
  {
    get_request("factory_reset");
  }

  bool release_handle(std::string handle)
  {
    get_request("release_handle", { "" }, { KV("handle", handle) });
    return true;
  }

  ProtocolInfo get_protocol_info()
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

    return opi;
  }

  template <typename... Ts>
  bool set_parameter(const std::initializer_list<param_type> params)
  {
    return get_request_bool("set_parameter", { "" }, params);
  }

  template <typename... Ts>
  std::map<std::string, std::string> get_parameter(const Ts&... ts)
  {
    return get_request("get_parameter", { ts... }, { KV("list", ts...) });
  }

  std::int64_t get_parameter_int(const std::string param)
  {
    std::map<std::string, std::string> resp = get_parameter(param);
    if (resp.empty())
    {
      return std::numeric_limits<std::int64_t>::quiet_NaN();
    }
    return to_long(resp[param]);
  }

  float get_parameter_float(const std::string param)
  {
    std::map<std::string, std::string> resp = get_parameter(param);
    if (resp.empty())
    {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return to_float(resp[param]);
  }

  std::string get_parameter_str(const std::string param)
  {
    std::map<std::string, std::string> resp = get_parameter(param);
    if (resp.empty())
    {
      return std::string("");
    }
    return resp[param];
  }

  template <typename... Ts>
  bool reset_parameter(const Ts&... ts)
  {
    return get_request_bool("reset_parameter", { "" }, { KV("list", ts...) });
  }

  HandleInfo request_handle_tcp(const std::string port = "", const std::string packet_type = "")
  {
    param_map_type query;
    if (!port.empty())
      query["port"] = port;
    if (!packet_type.empty())
      query["packet_type"] = packet_type;
    auto resp = get_request("request_handle_tcp", { "handle", "port" }, query);

    HandleInfo handle_info;
    handle_info.hostname = hostname;
    handle_info.handle = resp["handle"];
    handle_info.port = resp["port"];
    return handle_info;
  }

  virtual HandleInfo request_handle_udp(const std::string host_ip, const std::string port,
                                        const std::string packet_type = "")
  {
    param_map_type query = { KV("address", host_ip), KV("port", port) };
    if (!packet_type.empty())
      query["packet_type"] = packet_type;
    auto resp = get_request("request_handle_udp", { "handle", "port" }, query);

    HandleInfo handle_info;
    handle_info.handle = resp["handle"];
    handle_info.port = resp["port"];
    return handle_info;
  }

  virtual ScanConfig get_scanoutput_config(std::string handle)
  {
    auto resp = get_request(
        "get_scanoutput_config",
        { "start_angle", "packet_type", "watchdogtimeout", "skip_scans", "watchdog", "max_num_points_scan" },
        { KV("handle", handle) });
    config.packet_type = resp["packet_type"];
    config.start_angle = to_long(resp["start_angle"]);
    config.watchdogtimeout = to_long(resp["watchdogtimeout"]);
    config.watchdog = (resp["watchdog"] == "off") ? false : true;
    config.skip_scans = to_long(resp["skip_scans"]);
    config.max_num_points_scan = to_long(resp["max_num_points_scan"]);
    return config;
  }

  bool set_scanoutput_config(std::string handle, ScanConfig config)
  {
    param_map_type query = { KV("handle", handle),
                             KV("start_angle", config.start_angle),
                             KV("packet_type", config.packet_type),
                             KV("max_num_points_scan", config.max_num_points_scan),
                             KV("watchdogtimeout", config.watchdogtimeout),
                             KV("skip_scans", config.skip_scans),
                             KV("watchdog", config.watchdog ? "on" : "off") };
    auto resp = get_request("set_scanoutput_config", { "" }, query);
    return true;
  }
  bool start_scanoutput(std::string handle)
  {
    get_request("start_scanoutput", { "" }, { { "handle", handle } });
    return true;
  }

  bool stop_scanoutput(std::string handle)
  {
    return get_request_bool("stop_scanoutput", { "" }, { { "handle", handle } });
  }

  std::string get_scanoutput_config(std::string param, std::string handle)
  {
    auto resp = get_request("get_scanoutput_config", { param }, { KV("handle", handle) });
    return resp[param];
  }

  bool feed_watchdog(std::string handle)
  {
    return get_request_bool("feed_watchdog", { "" }, { { "handle", handle } });
  }

  virtual std::string get_product()
  {
    return std::string("");
  }

  virtual ScanParameters get_scan_parameters(int start_angle = 0)
  {
    return ScanParameters();
  }

  virtual void handle_reconfig(pf_driver::PFDriverR2000Config& config, uint32_t level)
  {
  }

  virtual void handle_reconfig(pf_driver::PFDriverR2300Config& config, uint32_t level)
  {
  }
};

#endif
