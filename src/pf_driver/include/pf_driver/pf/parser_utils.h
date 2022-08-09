#pragma once

#include <boost/algorithm/string/split.hpp>
#include <vector>
#include <string>

namespace parser_utils
{

inline const std::vector<std::string> split(std::string str, const char delim = ';')
{
  std::vector<std::string> results;
  boost::split(results, str, [delim](char c) { return c == delim; });
  return results;
}

inline std::int64_t to_long(const std::string& s)
{
  std::int64_t int_val = 0;
  try
  {
    int_val = std::stoll(s);
  }
  catch (std::exception& e)
  {
    std::cerr << "conversion of data from string failed: " << s << std::endl;
    return std::numeric_limits<std::int64_t>::quiet_NaN();
  }
  return int_val;
}

inline float to_float(const std::string& s)
{
  float float_val = 0;
  try
  {
    float_val = std::stof(s);
  }
  catch (std::exception& e)
  {
    std::cerr << "conversion of data from string failed " << s << std::endl;
    return std::numeric_limits<float>::quiet_NaN();
  }
  return float_val;
}

} // namespace
