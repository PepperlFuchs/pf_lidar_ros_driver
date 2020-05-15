#include "pf_driver/pf/pf_interface.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    // TODO: Let this be handled by rosparams
    if(argc < 3)
    {
        ROS_ERROR("Please provide the IP address and the port");
        return -1;
    }
    std::string transport = argv[1];
    std::string ip = argv[2];
    std::string port = argv[3];

    ros::init(argc, argv, "pf_driver");
    ros::NodeHandle nh;

    std::unique_ptr<Connection> connection;
    if(transport == "udp")
        connection = std::make_unique<UDPConnection>(ip);  // need a factory for this?
    else if(transport == "tcp")
        connection = std::make_unique<TCPConnection>(ip);
    else
    {
        ROS_ERROR("Incorrect transport option.");
        return -1;
    }
    PFInterface pf_interface(std::move(connection));
    if(!pf_interface.init())
    {
        ROS_ERROR("Unable to initialize device");
        return -1;
    }
    pf_interface.start_transmission();
    std::this_thread::sleep_for(std::chrono::seconds(3));

    ros::spin();
    pf_interface.stop_transmission();
    return 0;
}
