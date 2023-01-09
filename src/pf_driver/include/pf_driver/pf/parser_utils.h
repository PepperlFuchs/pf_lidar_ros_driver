#pragma once

#include <string>
#include <vector>

namespace parser_utils
{
const std::vector<std::string> split(const std::string& str, const char delim = ';');

std::int64_t to_long(const std::string& s);

float to_float(const std::string& s);

}  // namespace parser_utils
