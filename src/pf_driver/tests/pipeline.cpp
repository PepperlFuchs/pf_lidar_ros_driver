#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include "pf_driver/ros/laser_scan_publisher.h"
#include "pf_driver/pf/pf_interface.h"
#include "pf_driver/communication/tcp_transport.h"
#include "pf_driver/test_helper.h"
#include "pf_driver/communication/transport.h"

void connection_cb()
{
  std::cout << "connection failure" << std::endl;
}

std::unique_ptr<Pipeline> get_pipeline(std::unique_ptr<Transport> transport, std::shared_ptr<Reader<PFPacket>> reader,
                                       std::shared_ptr<std::mutex> net_mtx,
                                       std::shared_ptr<std::condition_variable> net_cv, bool net_fail)
{
  auto logger = rclcpp::get_logger("pf_pipeline");

  std::shared_ptr<Parser<PFPacket>> parser = std::unique_ptr<Parser<PFPacket>>(new PFR2000_C_Parser);
  std::shared_ptr<Writer<PFPacket>> writer =
      std::shared_ptr<Writer<PFPacket>>(new PFWriter<PFPacket>(std::move(transport), parser, logger));

  return std::make_unique<Pipeline>(writer, reader, &connection_cb, net_mtx, net_cv, net_fail);
}

TEST(PFPipeline_TestSuite, testPipelineReadWrite)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<ScanParameters> params = std::make_shared<ScanParameters>();
  std::shared_ptr<ScanConfig> config = std::make_shared<ScanConfig>();
  config->start_angle = 1800000;
  config->max_num_points_scan = 0;
  config->packet_type = "C";
  config->watchdogtimeout = 60000;
  config->watchdog = true;

  std::shared_ptr<Reader<PFPacket>> reader =
      std::shared_ptr<PFPacketReader>(new LaserscanPublisher(config, params, "/scan", "scanner"));

  std::shared_ptr<std::mutex> net_mtx = std::make_shared<std::mutex>();
  std::shared_ptr<std::condition_variable> net_cv = std::make_shared<std::condition_variable>();
  bool net_fail = false;

  std::unique_ptr<Transport> transport = std::make_unique<TCPTransport>("127.0.0.1");
  transport->set_port("1234");

  if (transport->connect())
  {
    auto pipeline = get_pipeline(std::move(transport), reader, net_mtx, net_cv, net_fail);
    pipeline->start();

    std::unique_lock<std::mutex> net_lock(*net_mtx);
    net_cv->wait(net_lock, [&net_fail] { return net_fail; });

    std::cout << "pipeline shutdown" << std::endl;
    pipeline->terminate();
  }

  rclcpp::shutdown();
}