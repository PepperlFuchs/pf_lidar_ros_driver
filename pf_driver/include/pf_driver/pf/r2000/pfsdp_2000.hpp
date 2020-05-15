#pragma once

#include "pf_driver/pf/pfsdp_protocol.hpp"

class PFSDP_2000 : public PFSDPBase
{

public:
  PFSDP_2000(const utility::string_t &host) : PFSDPBase(host)
  {
  }

  virtual std::string get_start_angle_str()
  {
      return std::string("");
  }

  virtual std::string get_product()
  {
    return get_parameter_str("product");
  }
};