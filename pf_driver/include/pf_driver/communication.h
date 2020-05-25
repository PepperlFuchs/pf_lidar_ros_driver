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
  virtual bool read(boost::array<uint8_t, 4096> &buf, size_t &len) = 0;

  Transport(std::string address, transport_type typ) : address_(address), type_(typ), is_connected_(false) {}

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
  boost::asio::io_service io_service_;
  boost::thread io_service_thread_;
};

class TCPTransport : public Transport
{
public:
    TCPTransport(std::string address) : Transport(address, transport_type::tcp), socket_(io_service_) 
    {
      io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
    }

    ~TCPTransport()
    {
      disconnect();
    }

    virtual bool connect()
    {
      try
      {
        std::cout << "io_service " << io_service_.stopped() << std::endl;
        tcp::resolver resolver(io_service_);
        tcp::resolver::query query(address_, port_);
        tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        tcp::resolver::iterator end;

        boost::system::error_code error = boost::asio::error::host_not_found;
        while (error && endpoint_iterator != end)
        {
          socket_.close();
          socket_.connect(*endpoint_iterator++, error);
        }
        if (error)
        {
          throw boost::system::system_error(error);
        }
      }
      catch (std::exception& e)
      {
        std::cerr << e.what() << std::endl;
        return false;
      }
      is_connected_ = true;
      return true;
    }

    virtual bool disconnect()
    {
      std::cout << "disconnecting..." << std::endl;
      socket_.close();
      return true;
    }

    virtual bool read(boost::array<uint8_t, 4096> &buf, size_t &len)
    {
      boost::system::error_code error;
      len = socket_.read_some(boost::asio::buffer(buf), error);
      if (error == boost::asio::error::eof)
        return false; // Connection closed cleanly by peer.
      else if (error)
        return false;
      return true;
    }

private:
    tcp::socket socket_;
};

class UDPTransport : public Transport
{
public:
  UDPTransport(std::string address) : Transport(address, transport_type::tcp), socket_(io_service_) {}

  ~UDPTransport()
  {
    disconnect();
  }

  virtual bool connect()
  {
    udp::resolver resolver(io_service_);
    udp::resolver::query query(udp::v4(), address_);
    udp::endpoint receiver_endpoint = *resolver.resolve(query);

    socket_.open(udp::v4());
    port_ = std::to_string(socket_.local_endpoint().port());
    host_ip_ = socket_.local_endpoint().address().to_string();

    std::cout << host_ip_ << " " << port_ << std::endl;

    boost::array<char, 1> send_buf  = { 0 };
    socket_.send_to(boost::asio::buffer(send_buf), receiver_endpoint);

    is_connected_ = true;
    return true;
  }

  virtual bool disconnect()
  {
    std::cout << "disconnecting..." << std::endl;
    socket_.close();
  }

  virtual bool read(boost::array<uint8_t, 4096> &buf, size_t &len)
  {
    boost::system::error_code error;
    udp::endpoint sender_endpoint;
    len = socket_.receive_from(boost::asio::buffer(buf), sender_endpoint);
    if (error == boost::asio::error::eof)
      return false; // Connection closed cleanly by peer.
    else if (error)
      return false;
    return true;
  }

private:
  udp::socket socket_;
};

#endif
