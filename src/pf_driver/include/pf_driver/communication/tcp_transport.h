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

#include "pf_driver/communication/transport.h"

class TCPTransport : public Transport
{
public:
  TCPTransport(std::string address);

  ~TCPTransport();

  virtual bool connect();
  virtual bool disconnect();
  virtual bool read(boost::array<uint8_t, 4096>& buf, size_t& len);
  virtual bool readWithTimeout(boost::array<uint8_t, 4096>& buf, size_t& len, const uint32_t expiry_time);

private:
  std::unique_ptr<boost::asio::ip::tcp::socket> socket_;
};
