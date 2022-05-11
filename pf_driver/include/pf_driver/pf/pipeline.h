#pragma once

#include <chrono>
#include <thread>
#include <vector>
#include <memory>
#include <mutex>
#include <ros/ros.h>
#include "pf_driver/queue/readerwriterqueue.h"
#include "pf_driver/pf/pfsdp_protocol.hpp"

// R2000 / R2300 parser
template <typename T>
class Reader
{
public:
  virtual void read(std::shared_ptr<T> packet) = 0;
  virtual void set_scanoutput_config(ScanConfig config)
  {
  }
  virtual void set_scan_params(ScanParameters params)
  {
  }
  virtual bool start()
  {
    return false;
  }
  virtual bool stop()
  {
    return false;
  }
};

// TCP / UDP
template <typename T>
class Writer
{
public:
  virtual bool get(std::vector<std::unique_ptr<T>>& packets) = 0;
  virtual bool start() = 0;
  virtual bool stop() = 0;
};

template <typename T>
class Pipeline
{
public:
  Pipeline(std::shared_ptr<Writer<T>> writer, std::shared_ptr<Reader<T>> reader, std::function<void()> func)
    : writer_(writer), reader_(reader), shutdown(func), running_(false), shutdown_(false), queue_{ 100 }
  {
  }

  bool start()
  {
    if (running_)
      return true;

    if (!writer_->start() || !reader_->start())
    {
      ROS_ERROR("Unable to establish connection");
      return false;
    }

    running_ = true;
    shutdown_ = false;
    // ROS_INFO("Starting read-write pipeline!");

    reader_thread_ = std::thread(&Pipeline::run_reader, this);
    writer_thread_ = std::thread(&Pipeline::run_writer, this);
    return true;
  }

  void terminate()
  {
    shutdown_ = true;
    // ROS_INFO("Stopping read-write pipeline!");
    running_ = false;

    writer_->stop();
    reader_->stop();

    if (reader_thread_.joinable() && writer_thread_.joinable())
    {
      reader_thread_.join();
      writer_thread_.join();
    }
  }

  bool is_running()
  {
    return running_;
  }

  void on_shutdown()
  {
    if (shutdown && !shutdown_)
    {
      shutdown_ = true;
      shutdown();
    }
  }

private:
  moodycamel::BlockingReaderWriterQueue<std::unique_ptr<T>> queue_;  // the queue basically stored scan data
  std::shared_ptr<Reader<T>> reader_;
  std::shared_ptr<Writer<T>> writer_;
  std::function<void()> shutdown;
  std::atomic<bool> running_, shutdown_;
  std::thread reader_thread_, writer_thread_;
  std::mutex mutex_;

  void run_writer()
  {
    std::vector<std::unique_ptr<T>> packets;
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
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    writer_->stop();
    running_ = false;
    reader_->stop();
    on_shutdown();
  }

  void run_reader()
  {
    std::unique_ptr<T> packet;
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
    reader_->stop();
    running_ = false;
    writer_->stop();
    on_shutdown();
  }
};
