#include "pf_driver/pf/pf_interface.h"
// #include <ros/ros.h>

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

    // ros::init(argc, argv, "pf_driver");
    // ros::NodeHandle nh;

    std::unique_ptr<Transport> transport;
    // if(transport_str == "udp")
    //     transport = std::make_unique<UDPTransport>(IP);
    // else if(transport_str == "tcp")
    transport = std::make_unique<TCPTransport>("169.254.253.4");
    // else
    // {
    //     ROS_ERROR("Incorrect transport option.");
    //     return -1;
    // }
    transport->set_port("8000");
    transport->connect();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    // ros::spin();
    // PFInterface pf_interface(std::move(transport));
    // // PFInterface pf_interface(transport, IP);
    // if(!pf_interface.init())
    // {
    //     ROS_ERROR("Unable to initialize device");
    //     return -1;
    // }
    // pf_interface.start_transmission();
    // ros::spin();
    // pf_interface.stop_transmission();
    return 0;
}
