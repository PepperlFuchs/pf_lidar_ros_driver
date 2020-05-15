#pragma once

#include <chrono>
#include <thread>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include "pf_driver/queue/readerwriterqueue.h"

//R2000 / R2300 parser
template <typename T>
class Reader
{
public:
    virtual void read(std::shared_ptr<T> packet) = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
};

//TCP / UDP
template <typename T>
class Writer
{
public:
    virtual bool get(std::vector<std::unique_ptr<T>> &packets) = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
};

template <typename T>
class Pipeline
{
public:
    Pipeline(std::shared_ptr<Writer<T>> writer, std::shared_ptr<Reader<T>> reader) : writer_(writer), reader_(reader)
    {
        // TODO: init queue
    }

    bool start()
    {
        if(running_)
            return true;

        if(!writer_->start())
        {
            ROS_ERROR("Unable to establish connection");
            return false;
        }

        running_ = true;
        ROS_INFO("Starting read-write pipeline!");
        reader_thread_ = std::thread(&Pipeline::run_reader, this);
        writer_thread_ = std::thread(&Pipeline::run_writer, this);
        return true;
    }

    void stop()
    {
        if(!running_)
            return;

        ROS_INFO("Stopping read-write pipeline!");
        running_ = false;

        writer_->stop();
        reader_->stop();

        if(reader_thread_.joinable() && writer_thread_.joinable())
        {
            reader_thread_.join();
            writer_thread_.join();
        }
    }

private:
    moodycamel::BlockingReaderWriterQueue<std::unique_ptr<T>> queue_;   // the queue basically stored scan data
    std::shared_ptr<Reader<T>> reader_;
    std::shared_ptr<Writer<T>> writer_;
    std::atomic<bool> running_;
    std::thread reader_thread_, writer_thread_;

    void run_writer()
    {
        std::vector<std::unique_ptr<T>> packets;
        while(running_)
        {
            if(!writer_->get(packets))    // packets are already parsed here
            {
                break;
            }

            for(auto& p : packets)
            {
                if(!queue_.try_enqueue(std::move(p)))
                    ROS_ERROR("Queue overflow!");
            }
        }
        reader_->stop();
        running_ = false;
        writer_->stop();
    }

    void run_reader()
    {
        std::unique_ptr<T> packet;
        while(running_)
        {
            if (!queue_.wait_dequeue_timed(packet, std::chrono::milliseconds(5)))
            {
                //TODO: reader needs to handle if no packet is received
                continue;
            }
            reader_->read(std::move(packet));    // here the scans will be published
        }
        writer_->stop();
        running_ = false;
        reader_->stop();
    }
};
