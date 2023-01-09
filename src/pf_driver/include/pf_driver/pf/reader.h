#pragma once

#include <memory>

#include "pf_driver/pf/scan_config.h"
#include "pf_driver/pf/scan_parameters.h"

// R2000 / R2300 parser
template <typename T>
class Reader
{
public:
  virtual void read(std::shared_ptr<T> packet) = 0;
  virtual void set_scanoutput_config(ScanConfig config)
  {
  }
  virtual void set_scan_params(ScanParameters params)
  {
  }
  virtual bool start()
  {
    return false;
  }
  virtual bool stop()
  {
    return false;
  }
};
