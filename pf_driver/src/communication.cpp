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
  std::cout << "disconnecting..." << std::endl;
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
  std::cout << "disconnecting..." << std::endl;
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
