#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include "pf_driver/pf/pf_interface.h"

TEST(R2000Packet_TestSuite, testParseR2000Packet)
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("r2000_packet_parser");
    RCLCPP_INFO(node->get_logger(), "Testing packet parsing");

    PFInterface pf_interface(node);
    rclcpp::shutdown();
}