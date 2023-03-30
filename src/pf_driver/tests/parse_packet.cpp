#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include "pf_driver/pf/pf_interface.h"

TEST(R2000Packet_TestSuite, testParseR2000Packet)
{
  rclcpp::init(0, nullptr);

  // init variables for parsing
  uint8_t* buf; // read from dump file
  size_t buf_len = 1404;
  rclcpp::Logger logger = rclcpp::get_logger("r2000_packet_parser");
  std::vector<std::unique_ptr<PFPacket>> results;
  size_t used = 0;

  std::shared_ptr<Parser<PFPacket>> parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_C_Parser);
  bool isParsed = false;

  try{
    parser->parse(buf, buf_len, results, used, logger);
  } catch(std::exception &e) {
    RCLCPP_ERROR(logger, "Parsing failed");
  }

  EXPECT_EQ(isParsed, true);

  rclcpp::shutdown();
}
