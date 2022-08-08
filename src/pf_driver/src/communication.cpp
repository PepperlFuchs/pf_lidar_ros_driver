#include "pf_driver/communication.h"

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
  boost::system::error_code error;
  udp::endpoint sender_endpoint;

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
  boost::system::error_code error;
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
