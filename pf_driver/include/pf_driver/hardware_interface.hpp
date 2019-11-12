#ifndef PF_DRIVER_HARDWARE_INTERFACE_H
#define PF_DRIVER_HARDWARE_INTERFACE_H

#include <condition_variable>
#include <deque>
#include <mutex>

#include "pf_driver/communication.hpp"
#include "pf_driver/data_parser.hpp"
#include "pf_driver/data_type.h"

// Need a class where the LaserScan msg is populated by ScanData
// Probably same class should parse buffer to header and scan data

template <typename ConnectionType>
class HardwareInterface
{
public:
  HardwareInterface(std::string address, std::string port, DataParser *parser)
  {
    this->connection = new ConnectionType(address, port);
    this->parser = parser;  // probably pass mutex, and conditional var?

    connection->set_handle_read(&DataParser::parse_data, parser);
    is_connected = false;

    scans.resize(4);
  }

  bool connect()
  {
    if (!connection->connect())
    {
      std::cerr << "ERROR: unable to connect to device." << std::endl;
      is_connected = false;
      return false;
    }
    connection->start_read(1500);

    is_connected = true;
    return true;
  }

  void disconnect()
  {
    connection->disconnect();
    is_connected = false;
  }

  bool isConnected()
  {
    return connection->is_connected();
  }

  const std::string get_port()
  {
    return connection->get_port();
  }

  const std::string get_host_ip()
  {
    return connection->get_host_ip();
  }

  void set_port(std::string port)
  {
    connection->set_port(port);
  }

  // device specific interface should take care of feeding watchdog, etc
  std::vector<ScanData> get_scan(std::vector<int> layers)
  {
    std::vector<ScanData> scan_layers(layers.size());
    std::unique_lock<std::mutex> lock(data_mutex);
    for (int i = 0; i < layers.size(); i++)
    {
      if (!layers[i])
      {
        continue;
      }
      while (scans[i].size() < 2)
      {
        data_notifier.wait_for(lock, std::chrono::seconds(1));
      }
      // ScanData data;
      if (scans[i].size() >= 2)
      {
        scan_layers[i] = ScanData(std::move(scans[i].front()));
        scans[i].pop_front();
      }
    }
    return scan_layers;
  }

protected:
  Connection *connection;
  DataParser *parser;

  std::vector<std::deque<ScanData>> scans;

  std::condition_variable data_notifier;
  std::mutex data_mutex;

  bool is_connected;
};

#endif