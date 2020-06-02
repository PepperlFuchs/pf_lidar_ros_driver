#pragma once

#include "pf_driver/pf/pipeline.h"
#include "pf_driver/pf/pf_parser.h"
#include "pf_driver/communication.h"


template <typename T>
class PFWriter : public Writer<T>
{
public:
    PFWriter(std::unique_ptr<Transport> &&transport, std::shared_ptr<Parser<T>> parser) : transport_(std::move(transport)), parser_(parser)
    {
    }

    virtual bool start()
    {
        if(transport_)
        {
            std::cout << "writer is connecting.." << transport_->is_connected() <<  std::endl;
            if(transport_->is_connected())
                return true;

            
            return transport_->connect();
        }
        return false;
    }

    virtual bool stop()
    {
        if(transport_ && transport_->is_connected()) {
            return transport_->disconnect();
        }
        return false;
    }

    virtual bool get(std::vector<std::unique_ptr<T>> &packets)
    {
        // uint8_t buf[4096];
        boost::array<uint8_t, 4096> buf;
        size_t read = 0;
        if(transport_->read(buf, read))
        {
            parser_->parse(buf.data(), read, packets);
            return true;
        }
        return false;
    }

private:
    std::unique_ptr<Transport> transport_;;
    std::shared_ptr<Parser<T>> parser_;
};
