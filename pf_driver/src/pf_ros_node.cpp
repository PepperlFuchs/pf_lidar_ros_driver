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

#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include "pf_driver/driver.hpp"

class PF_Node
{
public:
    //! Initialize and connect to laser range finder
    PF_Node() : nh_("~")
    {
        driver = 0;

        nh_.param("frame_id", frame_id, std::string("/scan"));
        nh_.param("scanner_ip", scanner_ip, std::string(""));
        nh_.param("scan_frequency", scan_frequency, 100);
        nh_.param("samples_per_scan", samples_per_scan, 500);
        nh_.param("major_version", major_version, 0);
        nh_.param("connection_type", connection_type, 1);

        if (scanner_ip == "")
        {
            std::cerr << "IP of laser range finder not set!" << std::endl;
            return;
        }

        if (!connect(major_version, connection_type))
            return;

        scan_publisher = nh_.advertise<sensor_msgs::LaserScan>("scan", 100);
        cmd_subscriber = nh_.subscribe("control_command", 100, &PF_Node::cmdMsgCallback, this);
        get_scan_data_timer = nh_.createTimer(ros::Duration(1 / (2 * std::atof(driver->getParametersCached().at("scan_frequency").c_str()))), &PF_Node::getScanData, this);
    }

private:
    bool connect(int major_version, int connection_type)
    {
        delete driver;

        driver = new PFDriver();
        std::cout << "Connecting to scanner at " << scanner_ip << " ... ";
        if (driver->connect(scanner_ip, 80, major_version))
            std::cout << "OK" << std::endl;
        else
        {
            std::cout << "FAILED!" << std::endl;
            std::cerr << "Connection to scanner at " << scanner_ip << " failed!" << std::endl;
            return false;
        }

        driver->setScanFrequency(scan_frequency);
        driver->setSamplesPerScan(samples_per_scan);
        auto params = driver->getParameters();
        std::cout << "Current scanner settings:" << std::endl;
        std::cout << "============================================================" << std::endl;
        for (const auto &p : params)
            std::cout << p.first << " : " << p.second << std::endl;
        std::cout << "============================================================" << std::endl;

        std::cout << "Starting capturing: ";
        if (driver->startCapturingUDP())
            std::cout << "OK" << std::endl;
        else
        {
            std::cout << "FAILED!" << std::endl;
            return false;
        }
        return true;
    }

    void getScanData(const ros::TimerEvent &e)
    {
        auto scandata = driver->getFullScan();
        if (scandata.amplitude_data.empty() || scandata.distance_data.empty())
            return;

        sensor_msgs::LaserScan scanmsg;
        scanmsg.header.frame_id = frame_id;
        scanmsg.header.stamp = ros::Time::now();

        scanmsg.angle_min = -50 * M_PI / 180;
        scanmsg.angle_max = +50 * M_PI / 180;
        scanmsg.angle_increment = (100 * M_PI / 180) / float(scandata.distance_data.size());
        scanmsg.time_increment = 1 / 35.0f / float(scandata.distance_data.size());

        scanmsg.scan_time = 1 / std::atof(driver->getParametersCached().at("scan_frequency").c_str());
        scanmsg.range_min = std::atof(driver->getParametersCached().at("radial_range_min").c_str());
        scanmsg.range_max = std::atof(driver->getParametersCached().at("radial_range_max").c_str());

        scanmsg.ranges.resize(scandata.distance_data.size());
        scanmsg.intensities.resize(scandata.amplitude_data.size());
        for (std::size_t i = 0; i < scandata.distance_data.size(); i++)
        {
            scanmsg.ranges[i] = float(scandata.distance_data[i]) / 1000.0f;
            scanmsg.intensities[i] = scandata.amplitude_data[i];
        }
        scan_publisher.publish(scanmsg);
    }

    void cmdMsgCallback(const std_msgs::StringConstPtr &msg)
    {
        const std::string &cmd = msg->data;
        static const std::string set_scan_frequency_cmd("set scan_frequency=");
        static const std::string set_samples_per_scan_cmd("set samples_per_scan=");

        if (cmd.substr(0, set_scan_frequency_cmd.size()) == set_scan_frequency_cmd)
        {
            std::string value = cmd.substr(set_scan_frequency_cmd.size());
            int frequency = std::atoi(value.c_str());
            if (frequency == 50 || frequency == 100)
            {
                scan_frequency = frequency;
                driver->setScanFrequency(frequency);
            }
        }

        if (cmd.substr(0, set_samples_per_scan_cmd.size()) == set_samples_per_scan_cmd)
        {
            std::string value = cmd.substr(set_samples_per_scan_cmd.size());
            int samples = std::atoi(value.c_str());
            if (samples >= 0)
            {
                samples_per_scan = samples;
                driver->setSamplesPerScan(samples);
            }
        }
    }

    ros::NodeHandle nh_;
    ros::Timer get_scan_data_timer;
    ros::Publisher scan_publisher;
    ros::Subscriber cmd_subscriber;
    std::string frame_id;
    std::string scanner_ip;
    int major_version, connection_type;
    int scan_frequency;
    int samples_per_scan;
    PFDriver *driver;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pf_ros_node", ros::init_options::AnonymousName);
    new PF_Node();
    ros::spin();
    return 0;
}