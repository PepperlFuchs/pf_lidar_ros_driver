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

    Connection::Transport transport;
    if(transport_str == "udp")
        transport = Connection::Transport::UDP;
    else if(transport_str == "tcp")
        transport = Connection::Transport::TCP;
    else
    {
        ROS_ERROR("Incorrect transport option.");
        return -1;
    }
    // PFInterface pf_interface(std::move(connection));
    PFInterface pf_interface(transport, IP);
    if(!pf_interface.init())
    {
        ROS_ERROR("Unable to initialize device");
        return -1;
    }
    pf_interface.start_transmission();
    // std::this_thread::sleep_for(std::chrono::minutes(10));
    ros::spin();
    pf_interface.stop_transmission();
    return 0;
}
