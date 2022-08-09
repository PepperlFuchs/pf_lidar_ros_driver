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

#include <json/json.h>

#include "pf_driver/pf/http_helpers/curl_resource.h"
#include "pf_driver/pf/http_helpers/param_type.h"
#include "pf_driver/pf/http_helpers/param_map_type.h"
#include "pf_driver/pf/http_helpers/http_helpers.h"

class HTTPInterface
{
public:
  HTTPInterface(const std::string& host, const std::string& path = "") : host(host), base_path(path)
  {
  }

  const std::map<std::string, std::string>
  get(const std::vector<std::string>& json_keys, const std::string command,
      const std::initializer_list<param_type>& list = std::initializer_list<param_type>())
  {
    CurlResource res(host);
    res.append_path(base_path);
    res.append_path(command);
    res.append_query(list);
    return get_(json_keys, res);
  }

  const std::map<std::string, std::string> get(const std::vector<std::string>& json_keys, const std::string command,
                                               const param_map_type& params = param_map_type())
  {
    CurlResource res(host);
    res.append_path(base_path);
    res.append_path(command);
    res.append_query(params);
    return get_(json_keys, res);
  }

private:
  const std::map<std::string, std::string> get_(const std::vector<std::string>& json_keys, CurlResource& res)
  {
    Json::Value json_resp;
    std::map<std::string, std::string> json_kv;

    try
    {
      res.get(json_resp);
      json_kv[std::string("error_http")] = std::string("OK");
    }
    catch (curlpp::RuntimeError& e)
    {
      json_kv[std::string("error_http")] = std::string(e.what());
      return json_kv;
    }
    catch (curlpp::LogicError& e)
    {
      json_kv[std::string("error_http")] = std::string(e.what());
      return json_kv;
    }

    for (std::string key : json_keys)
    {
      try
      {
        if (json_resp[key].isArray())
          json_kv[key] = http_helpers::from_array(json_resp[key]);
        else
          json_kv[key] = json_resp[key].asString();
      }
      catch (std::exception& e)
      {
        json_kv[key] = "--COULD NOT RETRIEVE VALUE--";
      }
    }
    return json_kv;
  }

  const std::string host;
  const std::string base_path;
};
