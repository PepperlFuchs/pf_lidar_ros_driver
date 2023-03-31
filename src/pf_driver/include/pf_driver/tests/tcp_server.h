#include <iostream>
#include <boost/asio.hpp>
#include <fstream>

#include "pf_driver/tests/test_helper.h"

using namespace boost::asio;
using ip::tcp;
using std::cout;
using std::endl;
using std::string;

auto logger_server = rclcpp::get_logger("tcp_server");

void publish_data(tcp::socket& socket)
{
  std::string filename = "dump_r2000_C_large.txt";
  std::string filepath = get_dump_path() + filename;
  std::ifstream file(filepath, std::fstream::in);

  if (file.is_open())
  {
    std::string line = "";
    // read from file and socket write operation
    while (std::getline(file, line))
    {
      std::vector<uint8_t> vec = hex_to_bytes(line);
      uint8_t* buf = vec.data();
      boost::asio::write(socket, boost::asio::buffer(buf, vec.size()));
    }
  }
  else
  {
    RCLCPP_ERROR(logger_server, "Could not read file");
  }
}

void start_server(int port)
{
  boost::asio::io_service io_service;
  // listen for new connection
  tcp::acceptor acceptor_(io_service, tcp::endpoint(tcp::v4(), port));
  // socket creation
  tcp::socket socket_(io_service);
  // waiting for connection
  acceptor_.accept(socket_);
  // read operation
  RCLCPP_INFO(logger_server, "Client accepted");
  // write operation
  publish_data(socket_);
  RCLCPP_INFO(logger_server, "Sent dump data to client");
  ;
}
