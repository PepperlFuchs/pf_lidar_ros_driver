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

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <condition_variable>
#include <iostream>
#include <mutex>

class Connection
{
public:
  enum class Transport
  {
    TCP,
    UDP
  };
  Transport TRANSPORT;

  virtual bool read(uint8_t* buf, size_t buf_len, size_t& total) = 0;

  virtual bool is_connected()
  {
    return is_connected_;
  }

  virtual bool disconnect()
  {
    is_connected_ = false;
    try
    {
      close();
      io_service.stop();
      if (boost::this_thread::get_id() != io_service_thread.get_id())
        io_service_thread.join();
        return true;
    }
    catch (std::exception &e)
    {
      std::cerr << "Exception: " << e.what() << std::endl;
    }
    return false;
  }

  virtual const std::string get_port()
  {
    return this->port;
  }

  virtual void set_port(std::string port)
  {
    this->port = port;
  }

  virtual const std::string get_host_ip()
  {
    return this->host_ip;
  }

  virtual const std::string get_device_ip()
  {
    return this->device_ip;
  }


  // virtual ~Connection()
  // {
  //   disconnect();
  // }

  virtual bool connect(std::string port = "") = 0;
  virtual void close() = 0;

protected:
  boost::thread io_service_thread;
  boost::asio::io_service io_service;

  std::atomic<bool> is_connected_;
  double last_data_time;
  std::string device_ip, port, host_ip;
};

class TCPConnection : public Connection
{
public:
  TCPConnection(std::string IP) : tcp_socket(boost::asio::ip::tcp::socket(io_service))
  {
    this->device_ip = IP;
    this->TRANSPORT = Transport::TCP;
    is_connected_ = false;
  }

  ~TCPConnection()
  {
    disconnect();
  }

  bool connect(std::string port)
  {
    boost::asio::ip::tcp::resolver resolver(io_service);
    boost::asio::ip::tcp::resolver::query query(device_ip, this->port);
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    boost::asio::ip::tcp::resolver::iterator end;

    // tcp_socket = new boost::asio::ip::tcp::socket(io_service);
    boost::system::error_code error = boost::asio::error::host_not_found;

    ROS_INFO("trying to connect");

    // Iterate over endpoints and etablish connection
    while (error && endpoint_iterator != end)
    {
      // std::cout << "Trying " << endpoint_iterator->endpoint() << "...\n";
      tcp_socket.close();
      tcp_socket.connect(*endpoint_iterator++, error);
    }
    if (error)
    {
      ROS_ERROR("Connection error: %s", error.message().c_str());
      is_connected_ = false;
      return false;
    }

    is_connected_ = true;
    // this->port = port;
    return true;
  }

  void close()
  {
    // if (tcp_socket)
      tcp_socket.close();
  }

  virtual bool read(uint8_t* buf, size_t buf_len, size_t& total)
  {
    try
    {
      total = tcp_socket.read_some(boost::asio::buffer(buf, buf_len));
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
      return false;  // keep an eye on this
    }
    return true;
  }

private:
  boost::asio::ip::tcp::socket tcp_socket;
};

class UDPConnection : public Connection
{
public:
  UDPConnection(std::string IP)
  {
    this->device_ip = IP;
    // this->port = port;
    this->TRANSPORT = Transport::UDP;
    is_connected_ = false;
  }
  bool connect(std::string port)
  {
    try
    {
      udp_socket = new boost::asio::ip::udp::socket(
          io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0));
      udp_endpoint =
          boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(device_ip), atoi(this->port.c_str()));

      udp_socket->connect(udp_endpoint);

      this->port = std::to_string(udp_socket->local_endpoint().port());
      host_ip = udp_socket->local_endpoint().address().to_string();
    }
    catch (const boost::system::system_error &ex)
    {
      std::cerr << "ERROR: " << ex.code() << " " << ex.what() << std::endl;
      is_connected_ = false;
      return false;
    }
    is_connected_ = true;
    return true;
  }

  void close()
  {
    if (udp_socket)
      udp_socket->close();
  }

  virtual bool read(uint8_t* buf, size_t buf_len, size_t& total)
  {
    total = udp_socket->receive(boost::asio::buffer(buf, buf_len));
    return true;
  }

private:
  boost::asio::ip::udp::socket *udp_socket;
  boost::asio::ip::udp::endpoint udp_endpoint;
};

#endif
