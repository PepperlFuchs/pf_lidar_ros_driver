#pragma once

#include "pf_driver/pf/pipeline.h"
#include "pf_driver/pf/pf_parser.h"
#include "pf_driver/communication.h"


template <typename T>
class PFWriter : public Writer<T>
{
public:
    PFWriter(std::unique_ptr<Connection> &&connection, std::shared_ptr<PFParser<T>> &parser) : connection_(std::move(connection)), parser_(parser)
    {
    }

    virtual bool start()
    {
        ROS_INFO("starting writer");
        std::cout << connection_->is_connected() << std::endl;
        if(connection_ && !connection_->is_connected()) {
            ROS_INFO("Device not connected. Will try to connect now...");
            return connection_->connect();
        }
        return false;
    }

    virtual bool stop()
    {
        if(connection_ && connection_->is_connected()) {
            return connection_->disconnect();
        }
        return false;
    }

    virtual bool get(std::vector<std::unique_ptr<T>> &packets)
    {
        //BinParser
        //parser.parse
        uint8_t buf[4096];
        size_t read = 0;
        connection_->read(buf, sizeof(buf), read);
        if(read > 0)
        {
            std::cout << "data read: " << read << std::endl;
            return true;
        }
        return false;
    }

private:
    std::unique_ptr<Connection> connection_;
    std::shared_ptr<PFParser<T>> parser_;
};
