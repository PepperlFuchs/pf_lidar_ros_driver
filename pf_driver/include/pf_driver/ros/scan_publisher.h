#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "pf_driver/pf/reader.h"
#include "pf_driver/pf/pf_packet.h"
#include "pf_driver/queue/readerwriterqueue.h"

class ScanPublisher : public PFPacketReader
{
public:
    ScanPublisher(std::string scan_topic, std::string frame_id) : scan_publisher_(nh_.advertise<sensor_msgs::LaserScan>(scan_topic, 1)), header_publisher_(nh_.advertise<pf_driver::PFR2000Header>("/r2000_header", 1)), frame_id_(frame_id), queue_{100}, prev_num_packet_(0) 
    {
        params_ = {};
    }

    virtual void read(PFR2000Packet &packet);
    virtual void read(PFR2300Packet &packet);

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
        config_.start_angle = config.start_angle;
        config_.max_num_points_scan = config.max_num_points_scan;
        config_.skip_scans = config.skip_scans;
    }
    
    virtual void set_scan_params(ScanParameters &params)
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        params_.angular_fov = params.angular_fov;
        params_.radial_range_min = params.radial_range_min;
        params_.radial_range_max = params.radial_range_max;
        // params_.angle_min = 0;
        // params_.angle_max = 0;
        // params_.layers_enabled = params.layers_enabled;
    }

private:
    ros::NodeHandle nh_;
    std::string frame_id_;
    ros::Publisher scan_publisher_;
    ros::Publisher header_publisher_;
    moodycamel::BlockingReaderWriterQueue<sensor_msgs::LaserScanPtr> queue_;
    std::deque<sensor_msgs::LaserScanPtr> d_queue_;
    std::mutex q_mutex_;

    size_t prev_num_packet_;
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
