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

#pragma once

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/optional.hpp>

#include "pf_driver/communication/transport_type.h"

class Transport
{
public:
  virtual bool connect() = 0;
  virtual bool disconnect() = 0;
  virtual bool read(boost::array<uint8_t, 4096>& buf, size_t& len) = 0;
  virtual bool readWithTimeout(boost::array<uint8_t, 4096>& buf, size_t& len, const uint32_t expiry_time) = 0;

  Transport(std::string address, transport_type typ) : address_(std::move(address)), type_(typ), is_connected_(false)
  {
  }

  void set_port(std::string port)
  {
    port_ = std::move(port);
  }

  std::string get_port()
  {
    return port_;
  }

  std::string get_host_ip()
  {
    return host_ip_;
  }

  std::string get_device_ip()
  {
    return address_;
  }

  transport_type get_type()
  {
    return type_;
  }

  bool is_connected()
  {
    return is_connected_;
  }

protected:
  std::string address_;
  std::string host_ip_;
  std::string port_;
  bool is_connected_;
  transport_type type_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  std::shared_ptr<boost::asio::deadline_timer> timer_;
  boost::optional<boost::system::error_code> timer_result_;
};
