#ifndef PF_DRIVER_PF_INTERFACE_H
#define PF_DRIVER_PF_INTERFACE_H

#include <type_traits>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "pf_driver/communication.hpp"
#include "pf_driver/hardware_interface.hpp"
#include "pf_driver/pfsdp_protocol.hpp"
#include "pf_driver/r2000/data_type_r2000.hpp"
#include "pf_driver/r2300/data_type_r2300.hpp"

template <typename ConnectionType, typename ProtocolType, typename PacketHeader>
class PF_Interface : public HardwareInterface<ConnectionType>, public DataParser
{
public:
    PF_Interface(std::string address, std::string port, int major_version) : HardwareInterface<ConnectionType>(address, port, this)
    {
        protocol_interface = new ProtocolType(address);
        this->major_version = major_version;
    }

    ~PF_Interface()
    {
        disconnect();
    }

    bool connect()
    {
        if (!init_protocol())
        {
            return false;
        }
        handle_info = handle_connect();
        protocol_interface->start_scanoutput(handle_info.handle);
        feed_timeout = std::floor(std::max((handle_info.watchdog_timeout / 1000.0 / 3.0), 1.0));
        return true;
    }

    void disconnect()
    {
        protocol_interface->stop_scanoutput(handle_info.handle);
        protocol_interface->release_handle(handle_info.handle);
        HardwareInterface<ConnectionType>::disconnect();
    }

    void parse_data(std::string str) override
    {
        int start = find_packet_start(get_packet_type(), str);
        int len = str.length() - start;

        str.erase(str.begin(), str.end() - len);
        if (start >= 0 && str.size() >= get_header_size())
        {
            p_header = reinterpret_cast<PacketHeader *>((char *)str.c_str());

            std::uint16_t num_points = p_header->num_points_packet;
            int packet_num = p_header->packet_number;

            str.erase(str.begin(), str.end() - (str.size() - p_header->header_size));
            std::unique_lock<std::mutex> lock(HardwareInterface<ConnectionType>::data_mutex);

            if (packet_num == 1 || HardwareInterface<ConnectionType>::scans.empty())
            {
                HardwareInterface<ConnectionType>::scans.emplace_back();
                if (HardwareInterface<ConnectionType>::scans.size() > 100)
                {
                    HardwareInterface<ConnectionType>::scans.pop_front();
                    // std::cerr << "Too many scans in receiver queue: Dropping scans!" << std::endl;
                }
                HardwareInterface<ConnectionType>::data_notifier.notify_one();
            }
            ScanData &scandata = HardwareInterface<ConnectionType>::scans.back();

            for (int i = 0; i < num_points; i++)
            {
                std::string d = str.substr(0, sizeof(std::uint32_t));
                fill_scan_data(scandata, d);
                str.erase(str.begin(), str.end() - (str.size() - sizeof(std::uint32_t)));
            }
        }
    }

    void publish_scan(std::string frame_id)
    {
        auto scandata = HardwareInterface<ConnectionType>::get_scan();
        if (scandata.amplitude_data.empty() || scandata.distance_data.empty())
            return;

        sensor_msgs::LaserScan scanmsg;
        scanmsg.header.frame_id = frame_id;
        scanmsg.header.stamp = ros::Time::now();

        std::uint32_t fov = std::atof(parameters["angular_fov"].c_str()) / 2.0;
        scanmsg.angle_min = -fov * M_PI / 180;
        scanmsg.angle_max = +fov * M_PI / 180;

        scanmsg.angle_increment = (fov * M_PI / 90) / float(scandata.distance_data.size());
        scanmsg.time_increment = 1 / 35.0f / float(scandata.distance_data.size());

        scanmsg.scan_time = 1 / std::atof(parameters["scan_frequency"].c_str());
        scanmsg.range_min = std::atof(parameters["radial_range_min"].c_str());
        scanmsg.range_max = std::atof(parameters["radial_range_max"].c_str());

        scanmsg.ranges.resize(scandata.distance_data.size());
        scanmsg.intensities.resize(scandata.amplitude_data.size());
        for (std::size_t i = 0; i < scandata.distance_data.size(); i++)
        {
            scanmsg.ranges[i] = float(scandata.distance_data[i]) / 1000.0f;
            scanmsg.intensities[i] = scandata.amplitude_data[i];
        }
        scan_publisher.publish(scanmsg);
    }

    static int main(const std::string &device_name, int argc, char *argv[])
    {
        std::string node_name = device_name + "_driver";
        std::replace(node_name.begin(), node_name.end(), '/', '_');

        ros::init(argc, argv, node_name);
        ros::NodeHandle nh("~");

        std::string frame_id, scanner_ip;
        int scan_frequency, samples_per_scan, major_version;

        nh.param("frame_id", frame_id, std::string("/scan"));
        nh.param("scanner_ip", scanner_ip, std::string(""));
        nh.param("scan_frequency", scan_frequency, 100);
        nh.param("samples_per_scan", samples_per_scan, 500);
        nh.param("major_version", major_version, 0);

        PF_Interface<ConnectionType, ProtocolType, PacketHeader> pf_interface(scanner_ip, std::string("0"), major_version);
        pf_interface.init_publisher(nh);
        pf_interface.connect();

        ros::Rate pub_rate(2 * scan_frequency);
        while (ros::ok())
        {
            pf_interface.publish_scan(frame_id);
            pf_interface.feedWatchdog();
            ros::spinOnce();
            pub_rate.sleep();
        }

        return 0;
    }

    void feedWatchdog(bool feed_always = false)
    {
        const double current_time = std::time(0);

        if (feed_always || watchdog_feed_time < (current_time - feed_timeout))
        {
            if (!protocol_interface->feed_watchdog(handle_info.handle))
                std::cerr << "ERROR: Feeding watchdog failed!" << std::endl;
            watchdog_feed_time = current_time;
        }
    }

protected:
    HandleInfo handle_connect()
    {
        if (std::is_same<ConnectionType, TCPConnection>::value)
        {
            HandleInfo hi = protocol_interface->request_handle_tcp('C', 0);
            HardwareInterface<ConnectionType>::set_port(hi.port);
            try
            {
                HardwareInterface<ConnectionType>::connect();
            }
            catch (std::exception &e)
            {
                std::cerr << "Exception: " << e.what() << std::endl;
                return HandleInfo();
            }
            return hi;
        }
        else if (std::is_same<ConnectionType, UDPConnection>::value)
        {
            try
            {
                HardwareInterface<ConnectionType>::connect();
            }
            catch (std::exception &e)
            {
                std::cerr << "Exception: " << e.what() << std::endl;
                return HandleInfo();
            }
            return protocol_interface->request_handle_udp(HardwareInterface<ConnectionType>::get_port(), 'C', 0);
        }
    }

    bool init_protocol()
    {
        auto opi = protocol_interface->get_protocol_info();
        if (opi.version_major != major_version)
        {
            std::cerr << "ERROR: Could not connect to laser range finder!" << std::endl;
            return false;
        }
        if (opi.version_major != major_version)
        {
            std::cerr << "ERROR: Wrong protocol version (version_major=" << opi.version_major << ", version_minor=" << opi.version_minor << ")" << std::endl;
            return false;
        }
        protocol_info = opi;
        parameters = protocol_interface->get_parameter(protocol_interface->list_parameters());
        watchdog_feed_time = 0;
        return true;
    }

    void init_publisher(ros::NodeHandle nh)
    {
        scan_publisher = nh.advertise<sensor_msgs::LaserScan>("scan", 100);
    }

    void parser_data()
    {
    }

    void fill_scan_data(ScanData &scandata, std::string str)
    {
        std::uint32_t *data = reinterpret_cast<std::uint32_t *>((char *)str.c_str());

        std::uint32_t distance = (*data & 0x000FFFFF);
        std::uint32_t amplitude = ((*data & 0xFFF00000) >> 20);

        scandata.distance_data.push_back(distance);
        scandata.amplitude_data.push_back(amplitude);
    }

    int find_packet_start(std::string type, std::string str)
    {
        for (int i = 0; i < str.size() - 4; i++)
        {
            if (((unsigned char)str[i]) == 0x5c &&
                ((unsigned char)str[i + 1]) == 0xa2 &&
                ((unsigned char)str[i + 2]) == type[0] &&
                ((unsigned char)str[i + 3]) == type[1])
            {
                return i;
            }
        }
        return -1;
    }

    std::size_t get_header_size()
    {
        return sizeof(PacketHeader);
    }

    //should extend this in R2300 class
    std::string get_packet_type()
    {
        if (std::is_same<PacketHeader, PacketHeaderR2000>::value)
        {
            return "C";
        }
        else if (std::is_same<PacketHeader, PacketHeaderR2300>::value)
        {
            return "C1";
        }
    }

    ProtocolType *protocol_interface;
    PacketHeader *p_header;

    ros::Publisher scan_publisher;

    HandleInfo handle_info;
    ProtocolInfo protocol_info;
    std::map<std::string, std::string> parameters;

    int major_version;
    double watchdog_feed_time;
    double feed_timeout;
};

#endif