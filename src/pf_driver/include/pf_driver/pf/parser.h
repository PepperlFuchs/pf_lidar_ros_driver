#pragma once

template <typename T>
class Parser
{
public:
  virtual bool parse(uint8_t* buf, size_t buf_len, std::vector<std::unique_ptr<T>>& results, size_t& used) = 0;
};
