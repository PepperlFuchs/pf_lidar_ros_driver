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

#include <sstream>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <json/json.h>

#include "pf_driver/pf/http_helpers/param_map_type.h"
#include "pf_driver/pf/http_helpers/param_type.h"

class CurlResource
{
public:
  CurlResource(const std::string& host);

  void append_path(const std::string& path);

  void append_query(const std::initializer_list<param_type>& list, bool do_encoding = false);

  void append_query(const param_map_type& params, bool do_encoding = false);

  void get(Json::Value& json_resp);

  void print();

private:
  std::string url_;
  curlpp::Cleanup cleaner;
  curlpp::Easy request_;
  std::list<std::string> header_;
  std::stringstream response_;
};
