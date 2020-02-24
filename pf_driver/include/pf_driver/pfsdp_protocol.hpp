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

#include <boost/algorithm/string.hpp>
#include "pf_driver/http_helpers.hpp"

struct ProtocolInfo
{
  std::string protocol_name;          // protocol name, defaults to "pfsdp"
  int version_major;                  // major version of protocol
  int version_minor;                  // minor version of protocol
  std::vector<std::string> commands;  // list of available commands
};

struct HandleInfo
{
  static const int HANDLE_TYPE_TCP = 0;
  static const int HANDLE_TYPE_UDP = 1;

  int handle_type;
  std::string hostname;
  std::string port;
  std::string handle;
  char packet_type;
  int start_angle;
  bool watchdog_enabled;
  int watchdog_timeout;
};

class KV : public std::pair<std::string, std::string>
{
  template <typename T>
  static std::string make_list(bool first, const T &t)
  {
    std::stringstream s;
    if (!first)
      s << ";";
    s << t;
    return s.str();
  }

  template <typename T, typename... Ts>
  static std::string make_list(bool first, const T &t, Ts &&... list)
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
  KV(const std::string &k, const Ts &... list) : std::pair<std::string, std::string>(k, make_list(true, list...))
  {
  }
};

const std::vector<std::string> split(std::string str, const char delim = ';')
{
  std::vector<std::string> results;
  boost::split(results, str, [](char c) { return c == ';'; });
  return results;
}

std::int64_t to_long(const std::string &s)
{
  std::int64_t int_val = 0;
  try
  {
    int_val = std::stoll(s);
  }
  catch (std::exception &e)
  {
    std::cerr << "conversion of data from string failed" << std::endl;
    return std::numeric_limits<std::int64_t>::quiet_NaN();
  }
  return int_val;
}

float to_float(const std::string &s)
{
  float float_val = 0;
  try
  {
    float_val = std::stof(s);
  }
  catch (std::exception &e)
  {
    std::cerr << "conversion of data from string failed" << std::endl;
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

  const std::map<std::string, std::string>
  get_request(const std::string command, std::vector<std::string> json_keys = std::vector<std::string>(),
              std::initializer_list<param_type> query = std::initializer_list<param_type>())
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

  bool check_error(std::map<std::string, std::string> &mp, const std::string &err_code, const std::string &err_text,
                   const std::string &err_http)
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

public:
  PFSDPBase(const utility::string_t &host) : hostname(host), http_interface(new HTTPInterface(host, "cmd"))
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
    auto resp = get_request("get_protocol_info", { "protocol_name", "version_major", "version_minor", "commands" });
    ProtocolInfo opi;
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
  std::map<std::string, std::string> get_parameter(const Ts &... ts)
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
    return to_long(param);
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
  bool reset_parameter(const Ts &... ts)
  {
    return get_request_bool("reset_parameter", { "" }, { KV("list", ts...) });
  }

  HandleInfo request_handle_tcp(const char packet_type, const int start_angle)
  {
    auto resp = get_request("request_handle_tcp", { "handle", "port" },
                            { KV("packet_type", std::string(1, packet_type)), KV("start_angle", start_angle) });
    HandleInfo handle_info;
    handle_info.hostname = hostname;
    handle_info.handle = resp["handle"];
    handle_info.port = resp["port"];
    handle_info.packet_type = packet_type;
    handle_info.start_angle = start_angle;
    handle_info.watchdog_enabled = true;
    handle_info.watchdog_timeout = 60000;
    return handle_info;
  }

  HandleInfo request_handle_udp(const std::string host_ip, const std::string port, const char packet_type,
                                const int start_angle)
  {
    auto resp = get_request("request_handle_udp", { "handle", "port" }, { KV("address", host_ip), KV("port", port) });
    HandleInfo handle_info;
    handle_info.handle = resp["handle"];
    handle_info.port = resp["port"];
    handle_info.packet_type = packet_type;
    handle_info.start_angle = start_angle;
    handle_info.watchdog_enabled = true;
    handle_info.watchdog_timeout = 60000;
    return handle_info;
  }

  bool start_scanoutput(std::string handle)
  {
    return get_request_bool("start_scanoutput", { "" }, { { "handle", handle } });
  }

  bool stop_scanoutput(std::string handle)
  {
    return get_request_bool("stop_scanoutput", { "" }, { { "handle", handle } });
  }

  std::string get_scanoutput_config(std::string param, std::string handle) {
      auto resp = get_request("get_scanoutput_config", {param}, {KV("handle", handle)});
      return resp[param];
  }

  bool feed_watchdog(std::string handle)
  {
    return get_request_bool("feed_watchdog", { "" }, { { "handle", handle } });
  }

  // Protocol for R2300 -- should be a new class
  std::vector<int> get_layers_enabled()
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

  int get_parameter_i(std::string param)
  {
    auto resp = get_parameter(param.c_str());
    return stoi(resp[param]);
  }

  float get_parameter_f(std::string param)
  {
    auto resp = get_parameter(param.c_str());
    return stof(resp[param]);
  }

  std::string get_parameter_s(std::string param)
  {
    auto resp = get_parameter(param.c_str());
    return resp[param];
  }

  std::int32_t get_start_angle(std::string handle)
  {
    std::string angle_str = get_scanoutput_config(std::string("start_angle"), handle);
    std::int32_t start_angle = stof(angle_str);
    return start_angle / 10000;
  }

  std::pair<float, float> get_angle_min_max(std::string handle)
  {
    float measure_start_angle = get_parameter_i(std::string("measure_start_angle")) / 10000 * M_PI / 180.0;
    float measure_stop_angle = get_parameter_i(std::string("measure_stop_angle")) / 10000 * M_PI / 180.0;
    float start_angle = get_start_angle(handle) * M_PI / 180.0;
    std::string scan_direction = get_parameter_s(std::string("scan_direction"));

    float min = (measure_start_angle > start_angle) ? measure_start_angle : start_angle;
    float max = measure_stop_angle;
    return std::pair<float, float>(min, max);
  }

  bool set_scan_frequency(std::int32_t scan_frequency)
  {
    set_parameter({ KV("scan_frequency", scan_frequency) });
    return true;
  }
};

#endif