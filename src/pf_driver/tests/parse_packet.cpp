#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include "pf_driver/pf/pf_interface.h"
#include "pf_driver/tests/test_helper.h"

TEST(R2000Packet_TestSuite, testParseR2000Packet)
{
  rclcpp::init(0, nullptr);

  // init variables for parsing
  std::string filename = "dump_r2000_C.txt";
  std::vector<uint8_t> buffer = read_dump(filename);
  uint8_t* buf = buffer.data();
  size_t buf_len = buffer.size();
  rclcpp::Logger logger = rclcpp::get_logger("r2000_packet_parser");
  std::vector<std::unique_ptr<PFPacket>> results;
  size_t used = 0;

  pf_interfaces::msg::PFR2000Header header_;
  memcpy(&header_, buf, sizeof(pf_interfaces::msg::PFR2000Header));

  std::shared_ptr<Parser<PFPacket>> parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_C_Parser);
  bool isParsed = false;

  try
  {
    isParsed = parser->parse(buf, buf_len, results, used, logger);
  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR(logger, "Parsing failed");
  }

  EXPECT_EQ(isParsed, true);

  rclcpp::shutdown();
}
