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
    std::string transport_str = argv[1];
    std::string IP = argv[2];
    std::string port = argv[3];

    ros::init(argc, argv, "pf_driver");
    ros::NodeHandle nh;

    std::shared_ptr<Connection> connection;
    if(transport_str == "udp")
        connection = std::make_shared<UDPConnection>(IP);
    else if(transport_str == "tcp")
        connection = std::make_shared<TCPConnection>(IP);
    else
    {
        ROS_ERROR("Incorrect transport option.");
        return -1;
    }
    PFInterface pf_interface(connection);
    // PFInterface pf_interface(transport, IP);
    if(!pf_interface.init())
    {
        ROS_ERROR("Unable to initialize device");
        return -1;
    }
    pf_interface.start_transmission();
    ros::spin();
    pf_interface.stop_transmission();
    return 0;
}
