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

#pragma once

#define _TURN_OFF_PLATFORM_STRING
#include <iostream>
#include <memory>
#include <string>
#include <typeinfo>

//---CURL----//

#include <cstdlib>
#include <cerrno>
#include <sstream>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Exception.hpp>
#include <json/json.h>

using param_type = std::pair<std::string, std::string>;
using param_map_type = std::map<std::string, std::string>;

inline std::string from_array(Json::Value& val)
{
  std::string s = "";
  for (int i = 0; i < val.size() - 1; i++)
  {
    s += val[i].asString() + ";";
  }
  s += val[val.size() - 1].asString();
  return s;
}

class CurlResource
{
public:
  CurlResource(std::string host) : url_("")
  {
    url_ = "http://" + host;
    header_.push_back("Content-Type: application/json");
    request_.setOpt(new curlpp::options::HttpHeader(header_));
    request_.setOpt(curlpp::options::WriteStream(&response_));
  }

  void append_path(const std::string& path)
  {
    url_ += "/" + path;
  }

  void append_query(const std::initializer_list<param_type>& list, bool do_encoding = false)
  {
    url_ += "?";
    for (const auto& p : list)
    {
      url_ += p.first + "=" + p.second + "&";
    }
    url_.pop_back();
  }

  void append_query(const param_map_type& params, bool do_encoding = false)
  {
    url_ += "?";
    for (const auto& p : params)
    {
      url_ += p.first + "=" + p.second + "&";
    }
    url_.pop_back();
  }

  void get(Json::Value& json_resp)
  {
    request_.setOpt(curlpp::options::Url(url_));
    request_.perform();

    Json::Reader reader;
    reader.parse(response_, json_resp);
  }

  void print()
  {
    std::cout << url_ << std::endl;
  }

private:
  std::string url_;
  curlpp::Cleanup cleaner;
  curlpp::Easy request_;
  std::list<std::string> header_;
  std::stringstream response_;
};

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
          json_kv[key] = from_array(json_resp[key]);
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

#endif
