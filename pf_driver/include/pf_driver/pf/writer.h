#pragma once

#include "pf_driver/pf/pipeline.h"
#include "pf_driver/pf/pf_parser.h"
#include "pf_driver/communication.h"


template <typename T>
class PFWriter : public Writer<T>
{
public:
    PFWriter(std::unique_ptr<Connection> &&connection, std::shared_ptr<Parser<T>> parser) : connection_(std::move(connection)), parser_(parser)
    {
    }

    PFWriter(Connection::Transport transport, std::string IP, std::string port,  std::shared_ptr<Parser<T>> parser) : parser_(parser)
    {
        if(transport == Connection::Transport::UDP)
            connection_ = std::make_unique<UDPConnection>(IP);  // need a factory for this?
        else if(transport == Connection::Transport::TCP)
            connection_ = std::make_unique<TCPConnection>(IP);

        connection_->set_port(port);
    }

    virtual bool start()
    {
        if(connection_ && !connection_->is_connected()) {
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
        uint8_t buf[4096];
        size_t read = 0;
        if(connection_->read(buf, sizeof(buf), read))
        {
            parser_->parse(buf, read, packets);
            return true;
        }
        return false;
    }

private:
    std::unique_ptr<Connection> connection_;
    std::shared_ptr<Parser<T>> parser_;
};
