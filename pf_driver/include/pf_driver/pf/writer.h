#pragma once

#include "pf_driver/pf/pipeline.h"
#include "pf_driver/pf/pf_parser.h"
#include "pf_driver/communication.h"

template <typename T>
class PFWriter : public Writer<T>
{
public:
  PFWriter(std::unique_ptr<Transport>&& transport, std::shared_ptr<Parser<T>> parser)
    : transport_(std::move(transport)), parser_(parser)
  {
  }

  virtual bool start()
  {
    if (transport_)
    {
      if (transport_->is_connected())
        return true;

      return transport_->connect();
    }
    return false;
  }

  virtual bool stop()
  {
    if (transport_ && transport_->is_connected())
    {
      return transport_->disconnect();
    }
    return false;
  }

  virtual bool get(std::vector<std::unique_ptr<T>>& packets)
  {
    boost::array<uint8_t, 4096> buf;
    size_t read = 0;
    size_t used = 0;
    if (transport_->read(buf, read))
    {
      persistent_buffer_.insert(persistent_buffer_.end(), buf.begin(), buf.begin() + read);
      parser_->parse(persistent_buffer_.data(), persistent_buffer_.size(), packets, used);

      if (used)
      {
        if (used == persistent_buffer_.size())
          persistent_buffer_.clear();
        else
          persistent_buffer_.erase(persistent_buffer_.begin(), persistent_buffer_.begin() + used);
      }
      return true;
    }
    return false;
  }

private:
  std::unique_ptr<Transport> transport_;
  std::shared_ptr<Parser<T>> parser_;
  std::vector<uint8_t> persistent_buffer_;
};
