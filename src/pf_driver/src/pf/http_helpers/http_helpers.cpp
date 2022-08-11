#include "pf_driver/pf/http_helpers/http_helpers.h"

std::string http_helpers::from_array(const Json::Value& val)
{
  std::string s = "";
  for (int i = 0; i < val.size() - 1; i++)
  {
    s += val[i].asString() + ";";
  }
  s += val[val.size() - 1].asString();
  return s;
}
