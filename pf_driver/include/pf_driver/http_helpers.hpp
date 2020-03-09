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

#ifndef PF_DRIVER_HTTP_HELPER_H
#define PF_DRIVER_HTTP_HELPER_H

#define _TURN_OFF_PLATFORM_STRING
#include <cpprest/http_client.h>
#include <iostream>
#include <memory>
#include <string>
#include <typeinfo>

using namespace utility;            // Common utilities like string conversions
using namespace web;                // Common features like URIs.
using namespace web::http;          // Common HTTP functionality
using namespace web::http::client;  // HTTP client features

using param_type = std::pair<std::string, std::string>;

inline std::string to_string(web::json::value val)
{
  if (val.is_integer())
  {
    return std::to_string(val.as_integer());
  }
  if (val.is_double())
  {
    return std::to_string(val.as_double());
  }
  if (val.is_string())
  {
    return val.as_string();
  }
  if (val.is_array())
  {
    auto arr = val.as_array();
    std::string s = "";
    for (size_t i = 0; i < arr.size() - 1; i++)
    {
      s += to_string(arr[i]) + ";";
    }
    s += to_string(arr.at(arr.size() - 1));
    return s;
  }
  return std::string();
}

class Resource
{
  void set_host(const utility::string_t &host)
  {
    builder.set_host(host);
  }

  void set_scheme(const utility::string_t &scheme)
  {
    builder.set_scheme(scheme);
  }

  const std::string uri;
  uri_builder builder;

public:
  Resource(const utility::string_t &host)
  {
    set_scheme("http");
    set_host(host);
  }

  Resource append_query(const std::initializer_list<param_type> &list, bool do_encoding = false)
  {
    for (const auto &p : list)
    {
      builder.append_query(p.first, p.second, do_encoding);
    }
    return *this;
  }

  Resource set_path(const utility::string_t &path)
  {
    builder.set_path(path);
    return *this;
  }

  Resource append_path(const utility::string_t &path)
  {
    builder.append_path(path);
    return *this;
  }

  std::string to_string()
  {
    return builder.to_string();
  }
};

class HTTPInterface
{
public:
  HTTPInterface(const utility::string_t &host, const utility::string_t &path = "") : host(host), base_path(path)
  {
  }

  const std::map<std::string, std::string>
  get(const std::vector<std::string> &json_keys, const std::string command,
      const std::initializer_list<param_type> &list = std::initializer_list<param_type>())
  {
    const std::string uri = Resource(host).set_path(base_path).append_path(command).append_query(list).to_string();

    http_client client(uri);
    std::map<std::string, std::string> json_kv;
    json::value json_resp;
    http_response response;
    try
    {
      response = client.request(methods::GET).get();
      response.headers().set_content_type("application/json");
      json_resp = response.extract_json().get();
      json_kv[std::string("error_http")] = std::string("OK");
    }
    catch (const std::exception &e)
    {
      json_kv[std::string("error_http")] = std::string(e.what());
      return json_kv;
    }

    for (std::string key : json_keys)
    {
      try
      {
        json_kv[key] = to_string(json_resp.at(key));
      }
      catch (std::exception &e)
      {
        json_kv[key] = "--COULD NOT RETRIEVE VALUE--";
      }
    }
    return json_kv;
  }

private:
  const utility::string_t host;
  const utility::string_t base_path;
};

#endif
