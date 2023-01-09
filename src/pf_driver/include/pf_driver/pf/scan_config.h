#pragma once

#include <string>

struct ScanConfig
{
  bool watchdog = false;
  uint watchdogtimeout = 0;
  std::string packet_type = "";
  int start_angle = 0;
  uint max_num_points_scan = 0;
  uint skip_scans = 0;

  // void print()
  // {
  //   std::cout << "Scan output config:\n"
  //             << "watchdogtimeout: " << watchdogtimeout << "\n"
  //             << "packet_type: " << packet_type << "\n"
  //             << "start_angle: " << start_angle << "\n"
  //             << "max_num_points_scan:" << max_num_points_scan << "\n"
  //             << "skip_scan: " << skip_scans << std::endl;
  // }
};
