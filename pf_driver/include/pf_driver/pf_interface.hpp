#ifndef PF_DRIVER_PF_INTERFACE_H
#define PF_DRIVER_PF_INTERFACE_H

#include <dynamic_reconfigure/server.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <type_traits>

#include "pf_driver/communication.hpp"
#include "pf_driver/hardware_interface.hpp"
#include "pf_driver/pfsdp_protocol.hpp"
#include "pf_driver/r2000/data_type_r2000.hpp"
#include "pf_driver/r2300/data_type_r2300.hpp"

#include "pf_driver/PFDriverConfig.h"

#include <ros/serialization.h>

template <typename ConnectionType, typename ProtocolType, typename PacketHeader>
class PF_Interface : public HardwareInterface<ConnectionType>, public DataParser
{
public:
  PF_Interface(std::string address, std::string port, int major_version)
    : HardwareInterface<ConnectionType>(address, port, this)
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

    if (std::is_same<PacketHeader, PacketHeaderR2300>::value)
    {
      layers = protocol_interface->get_layers_enabled();
    }
    else if (std::is_same<PacketHeader, PacketHeaderR2000>::value)
    {
      layers.resize(1, 0);
      layers[0] = 1;
    }

    handle_info = handle_connect();
    protocol_interface->start_scanoutput(handle_info.handle);
    angle_min_max = protocol_interface->get_angle_min_max(handle_info.handle);
    feed_timeout = std::floor(std::max((handle_info.watchdog_timeout / 1000.0 / 3.0), 1.0));
    return true;
  }

  void disconnect()
  {
    protocol_interface->stop_scanoutput(handle_info.handle);
    protocol_interface->release_handle(handle_info.handle);
    HardwareInterface<ConnectionType>::disconnect();
  }

  void parse_data(std::basic_string<u_char> str) override
  {
    str.insert(0, remaining_data);
    remaining_data.clear();
    int start = find_packet_start(get_packet_type(), str);

    if (start >= 0 && str.size() >= get_header_size())
    {
      p_header = reinterpret_cast<PacketHeader *>((u_char *)str.c_str());

      std::uint16_t num_points = p_header->num_points_packet;
      std::uint16_t packet_size = p_header->packet_size;
      std::uint64_t timestamp = p_header->timestamp_raw;
      float angular_increment = (p_header->angular_increment / 10000.0) * M_PI / 180.0;
      int packet_num = p_header->packet_number;
      int layer_index = 0;

      if (std::is_same<PacketHeader, PacketHeaderR2300>::value)
      {
        layer_index = reinterpret_cast<PacketHeaderR2300 *>(p_header)->layer_index;
      }

      std::uint32_t *p_scan_data = (std::uint32_t *)&str[p_header->header_size];

      std::unique_lock<std::mutex> lock(HardwareInterface<ConnectionType>::data_mutex);

      if (packet_num == 1 || HardwareInterface<ConnectionType>::scans[layer_index].empty())
      {
        HardwareInterface<ConnectionType>::scans[layer_index].emplace_back();
        if (HardwareInterface<ConnectionType>::scans[layer_index].size() > 100)
        {
          HardwareInterface<ConnectionType>::scans[layer_index].pop_front();
          // std::cerr << "Too many scans in receiver queue: Dropping scans!" << std::endl;
        }
        HardwareInterface<ConnectionType>::data_notifier.notify_one();
      }
      ScanData &scandata = HardwareInterface<ConnectionType>::scans[layer_index].back();

      for (int i = 0; i < num_points; i++)
      {
        std::uint32_t data = p_scan_data[i];

        std::uint32_t distance = (data & 0x000FFFFFul);
        std::uint32_t amplitude = ((data & 0xFFF00000ul) >> 20);

        if (amplitude < 32)
        {
          amplitude = std::numeric_limits<std::uint32_t>::quiet_NaN();
        }

        scandata.distance_data.push_back(distance);
        scandata.amplitude_data.push_back(amplitude);
      }

      scandata.header.timestamp = timestamp;
      scandata.header.angular_increment = angular_increment;

      remaining_data = str.substr(packet_size, str.size());
    }
  }

  void copy_pointcloud(sensor_msgs::PointCloud2 &c1, sensor_msgs::PointCloud2 c2)
  {
    c1.height = c2.height;
    c1.width = c2.width;
    c1.is_bigendian = c2.is_bigendian;
    c1.point_step = c2.point_step;
    c1.row_step = c2.row_step;

    c1.fields = std::move(c2.fields);
    c1.data = std::move(c2.data);
  }

  // TODO: should move() the 2nd param
  void add_pointcloud(sensor_msgs::PointCloud2 &orig, sensor_msgs::PointCloud2 cloud)
  {
    pcl::PCLPointCloud2 p1, p2;
    pcl_conversions::toPCL(orig, p1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr p1_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // handle when point cloud is empty
    pcl::fromPCLPointCloud2(p1, *p1_cloud);

    pcl_conversions::toPCL(cloud, p2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr p2_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(p2, *p2_cloud);

    *p1_cloud += *p2_cloud;
    pcl::toROSMsg(*p1_cloud.get(), orig);
  }

  void publish_scan(std::string frame_id)
  {
    auto scandatas = HardwareInterface<ConnectionType>::get_scan(layers);
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = "/scan";
    cloud.header.stamp = ros::Time::now();

    for (int i = 0; i < layers.size(); i++)
    {
      if (!layers[i])
        continue;
      ScanData scandata = scandatas[i];
      if (scandata.amplitude_data.empty() || scandata.distance_data.empty())
        return;

      std::size_t num_points = scandata.distance_data.size();

      sensor_msgs::LaserScan scanmsg;
      scanmsg.header.frame_id = frame_id + "_" + std::to_string(i);
      scanmsg.header.stamp = ros::Time::now();

      float fov = angle_min_max.second - angle_min_max.first;
      scanmsg.angle_min = angle_min_max.first;
      scanmsg.angle_max = angle_min_max.second;

      float scan_time = 1 / std::atof(parameters["scan_frequency"].c_str());
      scanmsg.scan_time = scan_time;
      scanmsg.range_min = std::atof(parameters["radial_range_min"].c_str());
      scanmsg.range_max = std::atof(parameters["radial_range_max"].c_str());

      scanmsg.angle_increment = scandata.header.angular_increment;
      scanmsg.time_increment = (fov * scan_time) / (M_PI * 2.0) / num_points;

      scanmsg.ranges.resize(num_points);
      scanmsg.intensities.resize(num_points);
      for (std::size_t i = 0; i < num_points; i++)
      {
        scanmsg.ranges[i] = float(scandata.distance_data[i]) / 1000.0f;
        scanmsg.intensities[i] = scandata.amplitude_data[i];
      }
      scan_publishers[i].publish(scanmsg);

      sensor_msgs::PointCloud2 cloud_;
      if (tfListener_.waitForTransform(scanmsg.header.frame_id, "/scan",
                                       scanmsg.header.stamp +
                                           ros::Duration().fromSec(scanmsg.ranges.size() * scanmsg.time_increment),
                                       ros::Duration(1.0)))
      {
        projector_.transformLaserScanToPointCloud("/scan", scanmsg, cloud_, tfListener_);
        if (cloud.data.empty())
        {
          copy_pointcloud(cloud, cloud_);
        }
        else
        {
          add_pointcloud(cloud, cloud_);
        }
      }
    }
  }
  pcl_publisher.publish(cloud);
}

void reconfig_callback(pf_driver::PFDriverConfig &config, uint32_t level)
  {
    ROS_INFO("Reconfigure Request: %d", config.scan_frequency);
    protocol_interface->set_scan_frequency(config.scan_frequency);
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
    nh.param("major_version", major_version, 0);

    using PF_Device = PF_Interface<ConnectionType, ProtocolType, PacketHeader>;
    PF_Device pf_interface(scanner_ip, std::string("0"), major_version);
    if (std::is_same<PacketHeader, PacketHeaderR2300>::value)
    {
      pf_interface.init_publishers(nh, 4);
    }
    else if (std::is_same<PacketHeader, PacketHeaderR2000>::value)
    {
      pf_interface.init_publishers(nh, 1);
    }

    dynamic_reconfigure::Server<pf_driver::PFDriverConfig> param_server;
    param_server.setCallback(
        boost::bind(&PF_Device::reconfig_callback, &pf_interface, boost::placeholders::_1, boost::placeholders::_2));
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
      return protocol_interface->request_handle_udp(HardwareInterface<ConnectionType>::get_host_ip(),
                                                    HardwareInterface<ConnectionType>::get_port(), 'C', 0);
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
      std::cerr << "ERROR: Wrong protocol version (version_major=" << opi.version_major
                << ", version_minor=" << opi.version_minor << ")" << std::endl;
      return false;
    }
    protocol_info = opi;
    parameters = protocol_interface->get_parameter(protocol_interface->list_parameters());
    watchdog_feed_time = 0;
    return true;
  }

  void init_publishers(ros::NodeHandle nh, std::size_t num_layers)
  {
    scan_publishers.resize(num_layers);
    for (int i = 0; i < num_layers; i++)
    {
      std::string topic = "/scan_" + std::to_string(i);
      scan_publishers[i] = nh.advertise<sensor_msgs::LaserScan>(topic.c_str(), 100);
    }
    pcl_publisher = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 100);
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

  int find_packet_start(std::string type, std::basic_string<u_char> str)
  {
    for (int i = 0; i < str.size() - 4; i++)
    {
      if (((unsigned char)str[i]) == 0x5c && ((unsigned char)str[i + 1]) == 0xa2 &&
          ((unsigned char)str[i + 2]) == type[0] && ((unsigned char)str[i + 3]) == type[1])
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

  // should extend this in R2300 class
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

  std::vector<ros::Publisher> scan_publishers;
  ros::Publisher pcl_publisher;

  laser_geometry::LaserProjection projector_;
  tf::TransformListener tfListener_;

  HandleInfo handle_info;
  ProtocolInfo protocol_info;
  std::map<std::string, std::string> parameters;

  int major_version;
  double watchdog_feed_time;
  double feed_timeout;
  std::vector<int> layers;
  std::pair<float, float> angle_min_max;

  std::basic_string<u_char> remaining_data;
};

#endif