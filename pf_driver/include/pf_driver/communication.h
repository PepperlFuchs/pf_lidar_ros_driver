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
// #include "pf_driver/data_parser.hpp"

class Connection
{
public:
  enum class Transport
  {
    TCP,
    UDP
  };
  Transport TRANSPORT;

  virtual bool start_read(std::size_t n)
  {
    if(!handle_read)
      return false;
    async_read(n, &Connection::handle_packet);
    io_service_thread = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
    return true;
  }

  virtual bool is_connected()
  {
    return is_connected_;
  }

  virtual bool checkConnection()
  {
    if (!is_connected())
      return false;
    if ((std::time(0) - last_data_time) > 2)
    {
      disconnect();
      return false;
    }
    return true;
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

  // void set_handle_read(boost::function<void(DataParser *parser, std::basic_string<u_char> str)> h, DataParser *parser)
  // {
  //   handle_read = boost::bind(h, parser, boost::placeholders::_1);
  // }

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


  virtual ~Connection()
  {
    disconnect();
  }

  virtual bool connect(std::string port = "") = 0;
  virtual void close() = 0;

protected:
  std::basic_string<u_char> get_buffer_string(std::size_t n)
  {
    buf.commit(n);
    std::basic_string<u_char> s(boost::asio::buffers_begin(buf.data()), boost::asio::buffers_end(buf.data()));
    buf.consume(n);
    return s;
  }

  void handle_packet(const boost::system::error_code &ec, std::size_t n)
  {
    if (!ec)
    {
      std::cout << "waiting for data..." << std::endl;
      std::basic_string<u_char> str = get_buffer_string(n);
      std::cout << "data received: " << str.size() << " " << str.c_str() << std::endl;
      if (str.empty())
        return;
      handle_read(str);
      async_read(1500, &Connection::handle_packet);
    }
    else if (ec != boost::asio::error::eof)
    {
      std::cout << "Error: " << ec << "\n";
    }
  }

  boost::thread io_service_thread;
  boost::asio::io_service io_service;

  boost::asio::streambuf buf;

  boost::function<void(std::basic_string<u_char> str)> handle_read;
  virtual void async_read(
      std::size_t s, boost::function<void(Connection *conn, const boost::system::error_code &ec, std::size_t n)> h) = 0;

  std::atomic<bool> is_connected_;
  double last_data_time;
  std::string device_ip, port, host_ip;
};

class TCPConnection : public Connection
{
public:
  TCPConnection(std::string IP)
  {
    this->device_ip = IP;
    // this->port = port;
    this->TRANSPORT = Transport::TCP;

  }

  bool connect(std::string port)
  {
    boost::asio::ip::tcp::resolver resolver(io_service);
    boost::asio::ip::tcp::resolver::query query(device_ip, this->port);
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    boost::asio::ip::tcp::resolver::iterator end;

    tcp_socket = new boost::asio::ip::tcp::socket(io_service);
    boost::system::error_code error = boost::asio::error::host_not_found;

    ROS_INFO("trying to connect");

    // Iterate over endpoints and etablish connection
    while (error && endpoint_iterator != end)
    {
      std::cout << "Trying " << endpoint_iterator->endpoint() << "...\n";
      tcp_socket->close();
      tcp_socket->connect(*endpoint_iterator++, error);
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
    if (tcp_socket)
      tcp_socket->close();
  }

private:
  void
  async_read(std::size_t s,
             boost::function<void(Connection *conn, const boost::system::error_code &ec, std::size_t n)> handle_read)
  {
    boost::asio::streambuf::mutable_buffers_type bufs = buf.prepare(s);
    // https://www.boost.org/doc/libs/1_51_0/doc/html/boost_asio/reference/async_read/overload1.html
    boost::asio::async_read(
        *tcp_socket, boost::asio::buffer(bufs), boost::asio::transfer_at_least(1),
        boost::bind(handle_read, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
  }

  boost::asio::ip::tcp::socket *tcp_socket;
};

class UDPConnection : public Connection
{
public:
  UDPConnection(std::string IP)
  {
    this->device_ip = IP;
    // this->port = port;
    this->TRANSPORT = Transport::UDP;
  }
  bool connect(std::string port)
  {
    try
    {
      udp_socket = new boost::asio::ip::udp::socket(
          io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), atoi(this->port.c_str())));
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

private:
  void
  async_read(std::size_t s,
             boost::function<void(Connection *conn, const boost::system::error_code &ec, std::size_t n)> handle_packet)
  {
    std::cout << "async read" << std::endl;
    boost::asio::streambuf::mutable_buffers_type bufs = buf.prepare(s);
    udp_socket->async_receive_from(boost::asio::buffer(bufs), udp_endpoint,
                                   boost::bind(handle_packet, this, boost::asio::placeholders::error,
                                               boost::asio::placeholders::bytes_transferred));
  }

  boost::asio::ip::udp::socket *udp_socket;
  boost::asio::ip::udp::endpoint udp_endpoint;
};

#endif
