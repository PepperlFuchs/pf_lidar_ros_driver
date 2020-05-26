#include <exception>
#include "pf_driver/ros/scan_publisher.h"

void ScanPublisher::read(PFR2000Packet_A &packet)
{
    header_publisher_.publish(packet.header);
    to_msg_queue<PFR2000Packet_A>(packet);
}

void ScanPublisher::read(PFR2000Packet_B &packet)
{
    header_publisher_.publish(packet.header);
    to_msg_queue<PFR2000Packet_B>(packet);
}

void ScanPublisher::read(PFR2000Packet_C &packet)
{
    header_publisher_.publish(packet.header);
    to_msg_queue<PFR2000Packet_C>(packet);
}

void ScanPublisher::read(PFR2300Packet_C1 &packet)
{
    // header_publisher_.publish(packet.header);
    to_msg_queue<PFR2300Packet_C1>(packet);
}

void ScanPublisher::run_publisher()
{
    // while(ros::ok())
    // {
    //     sensor_msgs::LaserScanPtr msg;
    //     if(!queue_.try_dequeue(msg))
    //         continue;
    //     if(!msg)
    //         continue;
    //     publish_scan(msg);
    // }
}

void ScanPublisher::publish_scan(sensor_msgs::LaserScanPtr msg)
{
    ros::Time t = ros::Time::now();
    msg->header.stamp = t;
    scan_publisher_.publish(std::move(msg));
}

// What are validation checks required here?
// Skipped scans?
// Device errors?
template <typename T>
void ScanPublisher::to_msg_queue(T &packet)
{
    if(!check_status(packet.header.status_flags))
        return;

    sensor_msgs::LaserScanPtr msg;
    if(d_queue_.empty())
        d_queue_.emplace_back();
    else if(d_queue_.size() > 100)
        d_queue_.pop_front();
    if(packet.header.header.packet_number == 1)
    {
        msg = d_queue_.front();
        if(msg)
        {
            publish_scan(msg);
            d_queue_.pop_front();
        }

        msg.reset(new sensor_msgs::LaserScan());
        msg->header.frame_id = frame_id_;
        msg->header.seq = packet.header.header.scan_number;
        msg->scan_time = 1000.0 / packet.header.scan_frequency;
        msg->angle_increment = packet.header.angular_increment / 10000.0 * (M_PI / 180.0);

        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            msg->time_increment = (params_.angular_fov * msg->scan_time) / (M_PI * 2.0) / packet.header.num_points_scan;
            msg->angle_min = params_.angle_min;
            msg->angle_max = params_.angle_max;
            msg->range_min = params_.radial_range_min;
            msg->range_max = params_.radial_range_max;
        }

        msg->ranges.resize(packet.header.num_points_scan);
        d_queue_.push_back(msg);
    }
    msg = d_queue_.back();
    if(!msg)
        return;

    // errors in scan_number - not in sequence sometimes
    if(msg->header.seq != packet.header.header.scan_number)
        return;
    int idx = packet.header.first_index;
    for(int i = 0; i < packet.header.num_points_packet; i++)
    {
        float data;
        if(packet.distance[i] == 0xFFFFFFFF)
            data = std::numeric_limits<std::uint32_t>::quiet_NaN();
        else
            data = packet.distance[i] / 1000.0;
        msg->ranges[idx + i] = std::move(data);
    }
}

// check the status bits here with a switch-case
// Currently only for logging purposes only
bool ScanPublisher::check_status(uint32_t status_flags)
{
    // if(packet.header.header.scan_number > packet.)
    return true;
}
