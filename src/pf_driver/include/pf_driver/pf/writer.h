#pragma once

#include <vector>

// TCP / UDP
template <typename T>
class Writer
{
public:
  virtual bool get(std::vector<std::unique_ptr<T>>& packets) = 0;
  virtual bool start() = 0;
  virtual bool stop() = 0;
};
