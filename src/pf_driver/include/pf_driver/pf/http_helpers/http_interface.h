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

#include "pf_driver/pf/http_helpers/param_type.h"
#include "pf_driver/pf/http_helpers/param_map_type.h"

class CurlResource;

class HTTPInterface
{
public:
  HTTPInterface(std::string host, std::string path = "");

  const std::map<std::string, std::string>
  get(const std::vector<std::string>& json_keys, const std::string& command,
      const std::initializer_list<param_type>& list = std::initializer_list<param_type>());

  const std::map<std::string, std::string> get(const std::vector<std::string>& json_keys, const std::string& command,
                                               const param_map_type& params = param_map_type());

private:
  const std::map<std::string, std::string> get_(const std::vector<std::string>& json_keys, CurlResource& res);

  const std::string host;
  const std::string base_path;
};
