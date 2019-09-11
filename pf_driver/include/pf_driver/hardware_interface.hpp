#ifndef PF_DRIVER_HARDWARE_INTERFACE_H
#define PF_DRIVER_HARDWARE_INTERFACE_H

#include <deque>
#include <mutex>
#include <condition_variable>

#include "pf_driver/data_parser.hpp"
#include "pf_driver/data_type.h"
#include "pf_driver/communication.hpp"

//Need a class where the LaserScan msg is populated by ScanData
//Probably same class should parse buffer to header and scan data

template <typename ConnectionType>
class HardwareInterface
{
public:
    HardwareInterface(std::string address, std::string port, DataParser *parser)
    {
        this->connection = new ConnectionType(address, port);
        this->parser = parser; //probably pass mutex, and conditional var?

        connection->set_handle_read(&DataParser::parse_data, parser);
        is_connected = false;
    }

    bool connect()
    {
        connection->connect();
        connection->start_read(4096);

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

    void set_port(std::string port)
    {
        connection->set_port(port);
    }

    //device specific interface should take care of feeding watchdog, etc
    ScanData get_scan()
    {
        std::unique_lock<std::mutex> lock(data_mutex);
        while (scans.size() < 2)
        {
            data_notifier.wait_for(lock, std::chrono::seconds(1));
        }
        ScanData data;
        if (scans.size() >= 2)
        {
            data = ScanData(std::move(scans.front()));
            scans.pop_front();
        }
        return data;
    }

protected:
    Connection *connection;
    DataParser *parser;

    std::deque<ScanData> scans;

    std::condition_variable data_notifier;
    std::mutex data_mutex;

    bool is_connected;
};

#endif