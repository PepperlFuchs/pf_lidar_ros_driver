#include <iostream>

#include "pf_driver/communication/tcp_transport.h"

using boost::asio::ip::tcp;

TCPTransport::TCPTransport(std::string address) : Transport(address, transport_type::tcp)
{
  io_service_ = std::make_shared<boost::asio::io_service>();
  socket_ = std::make_unique<tcp::socket>(*io_service_);
  timer_ = std::make_shared<boost::asio::deadline_timer>(*io_service_.get());
}

TCPTransport::~TCPTransport()
{
  disconnect();
}

bool TCPTransport::connect()
{
  try
  {
    tcp::resolver resolver(*io_service_);
    tcp::resolver::query query(address_, port_);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    tcp::resolver::iterator end;

    boost::system::error_code error = boost::asio::error::host_not_found;
    while (error && endpoint_iterator != end)
    {
      socket_->close();
      socket_->connect(*endpoint_iterator++, error);
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

bool TCPTransport::disconnect()
{
  io_service_->stop();
  socket_->close();
  return true;
}

bool TCPTransport::read(boost::array<uint8_t, 4096>& buf, size_t& len)
{
  boost::system::error_code error;
  len = socket_->read_some(boost::asio::buffer(buf), error);
  if (error == boost::asio::error::eof)
    return false;  // Connection closed cleanly by peer.
  else if (error)
    return false;
  return true;
}

bool TCPTransport::readWithTimeout(boost::array<uint8_t, 4096>& buf, size_t& len, const uint32_t expiry_time)
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

  socket_->async_read_some(boost::asio::buffer(buf),
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
