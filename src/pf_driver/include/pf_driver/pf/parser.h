#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

template <typename T>
class Parser
{
public:
  virtual bool parse(uint8_t* buf, size_t buf_len, std::vector<std::unique_ptr<T>>& results, size_t& used,
                     rclcpp::Logger logger) = 0;
};
