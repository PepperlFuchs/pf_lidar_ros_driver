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

#ifndef PF_DRIVER_COMMUNICATION_H
#define PF_DRIVER_COMMUNICATION_H

#pragma once

#include <iostream>
#include <thread>
#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;
using boost::asio::ip::udp;

enum transport_type
{
  tcp,
  udp
};

class Transport
{
public:
  virtual bool connect() = 0;
  virtual bool disconnect() = 0;
  virtual bool read(boost::array<uint8_t, 4096>& buf, size_t& len) = 0;

  Transport(std::string address, transport_type typ) : address_(address), type_(typ), is_connected_(false)
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
};

class TCPTransport : public Transport
{
public:
  TCPTransport(std::string address) : Transport(address, transport_type::tcp)
  {
    io_service_ = std::make_shared<boost::asio::io_service>();
    socket_ = std::make_unique<tcp::socket>(*io_service_);
  }

  ~TCPTransport()
  {
    disconnect();
  }

  virtual bool connect();
  virtual bool disconnect();
  virtual bool read(boost::array<uint8_t, 4096>& buf, size_t& len);

private:
  std::unique_ptr<tcp::socket> socket_;
};

class UDPTransport : public Transport
{
public:
  UDPTransport(std::string address) : Transport(address, transport_type::udp)
  {
    io_service_ = std::make_shared<boost::asio::io_service>();
    socket_ = std::make_unique<udp::socket>(*io_service_, udp::endpoint(udp::v4(), 0));
  }

  ~UDPTransport()
  {
    disconnect();
  }

  virtual bool connect();
  virtual bool disconnect();
  virtual bool read(boost::array<uint8_t, 4096>& buf, size_t& len);

private:
  std::unique_ptr<udp::socket> socket_;
};

#endif
