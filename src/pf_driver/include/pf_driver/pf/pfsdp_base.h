// Copyright 2022 Fraunhofer IPA
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

#pragma once

#include <mutex>
#include <memory>
#include <functional>

#include "pf_driver/pf/http_helpers/http_interface.h"
#include "pf_driver/pf/http_helpers/param_type.h"
#include "pf_driver/pf/http_helpers/param_map_type.h"
#include "pf_driver/pf/handle_info.h"
#include "pf_driver/pf/scan_config.h"
#include "pf_driver/pf/scan_parameters.h"
#include "pf_driver/pf/protocol_info.h"
#include "pf_driver/pf/kv.h"

class HTTPInterface;

class PFSDPBase
{
private:
  using HTTPInterfacePtr = std::unique_ptr<HTTPInterface>;
  HTTPInterfacePtr http_interface;
  std::function<void()> handle_connection_failure;

  const std::map<std::string, std::string> get_request(const std::string& command,
                                                       const std::vector<std::string>& json_keys,
                                                       const std::initializer_list<param_type>& query);
  const std::map<std::string, std::string>
  get_request(const std::string& command, const std::vector<std::string>& json_keys = std::vector<std::string>(),
              const param_map_type& query = param_map_type());

  bool get_request_bool(const std::string& command,
                        const std::vector<std::string>& json_keys = std::vector<std::string>(),
                        const std::initializer_list<param_type>& query = std::initializer_list<param_type>());

  bool is_connection_failure(const std::string& http_error);

  bool check_error(std::map<std::string, std::string>& mp, const std::string& err_code, const std::string& err_text,
                   const std::string& err_http);

protected:
  std::shared_ptr<HandleInfo> info_;
  std::shared_ptr<ScanConfig> config_;
  std::shared_ptr<ScanParameters> params_;

public:
  PFSDPBase(std::shared_ptr<HandleInfo> info, std::shared_ptr<ScanConfig> config,
            std::shared_ptr<ScanParameters> params);

  void set_connection_failure_cb(std::function<void()> callback);

  const std::vector<std::string> list_parameters();

  bool reboot_device();

  void factory_reset();

  bool release_handle(const std::string& handle);

  ProtocolInfo get_protocol_info();

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

  std::int64_t get_parameter_int(const std::string& param);

  float get_parameter_float(const std::string& param);

  std::string get_parameter_str(const std::string& param);

  template <typename... Ts>
  bool reset_parameter(const Ts&... ts)
  {
    return get_request_bool("reset_parameter", { "" }, { KV("list", ts...) });
  }

  void request_handle_tcp(const std::string& port = "", const std::string& packet_type = "");

  virtual void request_handle_udp(const std::string& packet_type = "");

  virtual void get_scanoutput_config(const std::string& handle);

  bool set_scanoutput_config(const std::string& handle, const ScanConfig& config);

  bool update_scanoutput_config();

  bool start_scanoutput();

  bool stop_scanoutput(const std::string& handle);

  std::string get_scanoutput_config(const std::string& param, const std::string& handle);

  bool feed_watchdog(const std::string& handle);

  virtual std::string get_product();

  virtual std::string get_part();

  virtual void get_scan_parameters();

  virtual void setup_param_server();
};
