#include "pf_driver/pf/pipeline.h"

#include "pf_driver/pf/pf_packet/pf_packet.h"

Pipeline::Pipeline(std::shared_ptr<Writer<PFPacket>> writer, std::shared_ptr<Reader<PFPacket>> reader,
                   std::function<void()> func, std::shared_ptr<std::mutex> net_mtx,
                   std::shared_ptr<std::condition_variable> net_cv, bool& net_fail)
  : queue_{ 100 }
  , reader_(reader)
  , writer_(writer)
  , shutdown(func)
  , running_(false)
  , shutdown_(false)
  , net_mtx_(net_mtx)
  , net_cv_(net_cv)
  , net_fail_(net_fail)
{
}

bool Pipeline::start()
{
  if (running_)
    return true;

  if (!writer_->start() || !reader_->start())
  {
    ROS_ERROR("Unable to establish connection");
    return false;
  }

  running_ = true;
  // shutdown_ = false;
  // ROS_INFO("Starting read-write pipeline!");

  reader_thread_ = std::thread(&Pipeline::run_reader, this);
  writer_thread_ = std::thread(&Pipeline::run_writer, this);
  return true;
}

void Pipeline::terminate()
{
  // ROS_INFO("Stopping read-write pipeline!");
  writer_->stop();
  reader_->stop();

  running_ = false;

  if (reader_thread_.joinable() && writer_thread_.joinable())
  {
    reader_thread_.join();
    writer_thread_.join();
  }
}

bool Pipeline::is_running()
{
  return running_;
}

void Pipeline::on_shutdown()
{
  if (shutdown && !shutdown_)
  {
    shutdown_ = true;
    shutdown();
  }
}

void Pipeline::run_writer()
{
  std::vector<std::unique_ptr<PFPacket>> packets;
  while (running_)
  {
    if (!writer_->get(packets))  // packets are already parsed here
    {
      break;
    }
    for (auto& p : packets)
    {
      if (!queue_.try_enqueue(std::move(p)))
        ROS_DEBUG("Queue overflow!");
    }
    packets.clear();
  }
  writer_->stop();
  reader_->stop();

  running_ = false;

  // notify main thread about network failure
  {
    std::lock_guard<std::mutex> lock(*net_mtx_);
    net_fail_ = true;
  }
  net_cv_->notify_one();
}

void Pipeline::run_reader()
{
  std::unique_ptr<PFPacket> packet;
  while (running_)
  {
    // ROS_INFO("reader loop");
    // wait till next message is received from device with 1 second timeout
    if (!queue_.wait_dequeue_timed(packet, std::chrono::milliseconds(100)))
    {
      // TODO: reader needs to handle if no packet is received
      continue;
    }
    if (packet)
    {
      reader_->read(std::move(packet));  // here the scans will be published
    }
  }
}
