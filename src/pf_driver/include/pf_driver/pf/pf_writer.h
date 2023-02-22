#pragma once

#include "pf_driver/pf/writer.h"
#include "pf_driver/pf/pf_parser.h"
#include "pf_driver/communication/transport.h"

template <typename T>
class PFWriter : public Writer<T>
{
public:
  PFWriter(std::unique_ptr<Transport>&& transport, std::shared_ptr<Parser<T>> parser)
    : transport_(std::move(transport)), parser_(parser), is_running_(false)
  {
  }

  virtual bool start()
  {
    std::unique_lock<std::mutex> lck(mtx_);
    is_running_ = true;
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
    std::unique_lock<std::mutex> lck(mtx_);
    is_running_ = false;
    if (transport_ && transport_->is_connected())
    {
      return transport_->disconnect();
    }
    return false;
  }

  virtual bool get(std::vector<std::unique_ptr<T>>& packets)
  {
    std::unique_lock<std::mutex> lck(mtx_);
    if (!is_running_)
    {
      return false;
    }
    boost::array<uint8_t, 4096> buf;
    size_t read = 0;
    size_t used = 0;
    if (transport_->readWithTimeout(buf, read, 2))
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
  bool is_running_;
  std::mutex mtx_;
};
