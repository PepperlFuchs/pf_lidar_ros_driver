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

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>

#include "pf_driver/pf/http_helpers/param_map_type.h"
#include "pf_driver/pf/http_helpers/param_type.h"

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
