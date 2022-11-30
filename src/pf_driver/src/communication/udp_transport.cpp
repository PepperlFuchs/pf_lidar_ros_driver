#include <iostream>

#include "pf_driver/communication/udp_transport.h"

using boost::asio::ip::udp;

UDPTransport::UDPTransport(std::string address, std::string port) : Transport(address, transport_type::udp)
{
  io_service_ = std::make_shared<boost::asio::io_service>();
  udp::endpoint local_endpoint = local_endpoint = udp::endpoint(udp::v4(), stoi(port));

  socket_ = std::make_unique<udp::socket>(*io_service_, local_endpoint);
  timer_ = std::make_shared<boost::asio::deadline_timer>(*io_service_.get());
}

UDPTransport::~UDPTransport()
{
  disconnect();
}

bool UDPTransport::connect()
{
  udp::endpoint udp_endpoint = udp::endpoint(boost::asio::ip::address::from_string(address_), 0);
  socket_->connect(udp_endpoint);
  port_ = std::to_string(socket_->local_endpoint().port());
  host_ip_ = socket_->local_endpoint().address().to_string();

  is_connected_ = true;
  return true;
}

bool UDPTransport::disconnect()
{
  socket_->close();
  return true;
}

bool UDPTransport::read(boost::array<uint8_t, 4096>& buf, size_t& len)
{
  boost::system::error_code error;
  udp::endpoint sender_endpoint;
  len = socket_->receive_from(boost::asio::buffer(buf), sender_endpoint);
  if (error == boost::asio::error::eof)
    return false;  // Connection closed cleanly by peer.
  else if (error)
    return false;
  return true;
}

// https://stackoverflow.com/questions/13126776/asioread-with-timeout
bool UDPTransport::readWithTimeout(boost::array<uint8_t, 4096>& buf, size_t& len, const uint32_t expiry_time)
{
  timer_->expires_from_now(boost::posix_time::seconds(expiry_time));
  timer_->async_wait([this, &expiry_time](const boost::system::error_code& error) {
    timer_result_.reset(error);
    if (error.message() == "Success")
    {
      std::cout << "Time out: No packets received in " << expiry_time << " seconds" << std::endl;
    }
  });

  boost::optional<boost::system::error_code> read_result;
  udp::endpoint sender_endpoint;

  socket_->async_receive_from(boost::asio::buffer(buf), sender_endpoint,
                              [&len, &read_result](const boost::system::error_code& error, size_t received) {
                                len = received;
                                read_result.reset(error);
                              });
  bool success = false;
  while (io_service_->run_one())
  {
    if (read_result && read_result->value() == 0)
    {
      // packets received so cancel timer
      timer_->cancel();
      success = true;
    }
    else if (timer_result_)
    {
      // timeout
      socket_->cancel();
      success = false;
    }
  }
  io_service_->reset();
  return success;
}
