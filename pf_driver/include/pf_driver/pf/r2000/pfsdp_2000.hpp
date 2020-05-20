#pragma once

#include "pf_driver/pf/pfsdp_protocol.hpp"

class PFSDP_2000 : public PFSDPBase
{

public:
  PFSDP_2000(const utility::string_t &host) : PFSDPBase(host)
  {
  }

  virtual std::string get_product()
  {
    return get_parameter_str("product");
  }

  virtual ScanParameters get_scan_parameters(int start_angle)
  {
    auto resp = get_parameter("angular_fov", "radial_range_min", "radial_range_max");

    ScanParameters params;
    params.angular_fov = to_float(resp["angular_fov"]) * M_PI / 180.0;
    params.radial_range_max = to_float(resp["radial_range_max"]);
    params.radial_range_min = to_float(resp["radial_range_min"]);

    params.angle_min = start_angle / 10000.0f  * M_PI / 180.0;
    params.angle_max = params.angle_min + params.angular_fov;
    return params;
  }
};