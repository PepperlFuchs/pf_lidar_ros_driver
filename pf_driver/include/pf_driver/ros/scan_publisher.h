#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "pf_driver/pf/reader.h"
#include "pf_driver/pf/pf_packet.h"
#include "pf_driver/queue/readerwriterqueue.h"

class ScanPublisher : public PFPacketReader
{
public:
    ScanPublisher(std::string scan_topic, std::string frame_id) : scan_publisher_(nh_.advertise<sensor_msgs::LaserScan>(scan_topic, 1)), header_publisher_(nh_.advertise<pf_driver::PFR2000Header>("/r2000_header", 1)), frame_id_(frame_id)
    {
        // config_ = std::make_unique<ScanConfig>();
        // params_ = std::make_unique<ScanParameters>();
    }

    virtual void read(PFR2000Packet_A &packet);
    virtual void read(PFR2000Packet_B &packet);
    virtual void read(PFR2000Packet_C &packet);
    virtual void read(PFR2300Packet_C1 &packet);

    virtual bool start()
    {
        pub_thread_ = std::thread(&ScanPublisher::run_publisher, this);
        return true;
    }

    virtual bool stop()
    {
        pub_thread_.join();
        return true;
    }

    virtual void set_scanoutput_config(ScanConfig &config)
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        // config_ = config;
        config_.start_angle = config.start_angle;
        config_.max_num_points_scan = config.max_num_points_scan;
        config_.skip_scans = config.skip_scans;
    }
    
    virtual void set_scan_params(ScanParameters &params)
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        // params_ = params;
        params_.angular_fov = params.angular_fov;
        params_.radial_range_min = params.radial_range_min;
        params_.radial_range_max = params.radial_range_max;
        params_.angle_min = params.angle_min;
        params_.angle_max = params.angle_max;
        // params_.layers_enabled = params.layers_enabled;
        // params_.layers_enabled[0] = params.layers_enabled[0];
        // params_.layers_enabled[1] = params.layers_enabled[1];
        // params_.layers_enabled[2] = params.layers_enabled[2];
        // params_.layers_enabled[3] = params.layers_enabled[3];
        // std::copy(params.layers_enabled.begin(), params.layers_enabled.end(), params_.layers_enabled.begin());
    }

private:
    ros::NodeHandle nh_;
    std::string frame_id_;
    ros::Publisher scan_publisher_;
    ros::Publisher header_publisher_;
    // moodycamel::BlockingReaderWriterQueue<sensor_msgs::LaserScanPtr> queue_;
    std::deque<sensor_msgs::LaserScanPtr> d_queue_;
    std::mutex q_mutex_;

    std::thread pub_thread_;
    std::mutex config_mutex_;
    ScanConfig config_;
    ScanParameters params_;

    bool check_status(uint32_t status_flags);

    template <typename T>
    void to_msg_queue(T &packet);
    void run_publisher();
    void publish_scan(sensor_msgs::LaserScanPtr msg);
};
