#pragma once

#pragma pack(push, sp, 1)
struct ScanParameters
{
  double angular_fov = 0.0;
  double radial_range_min = 0.0;
  double radial_range_max = 0.0;
  double angle_min = 0.0;
  double angle_max = 0.0;
  uint16_t layers_enabled = 0;
  double scan_freq = 0.0;        // needed to calculate scan resolution in R2300
  uint16_t h_enabled_layer = 0;  // highest enabled layer
  bool apply_correction = true;

  // void print()
  // {
  //   std::cout << "Scan parameters:\n"
  //             << "angular_fov: " << angular_fov << "\n"
  //             << "radial_range_min: " << radial_range_min << "\n"
  //             << "radial_range_max: " << radial_range_max << "\n"
  //             << "angle_min: " << angle_min << "\n"
  //             << "angle_max: " << angle_max << "\n"
  //             << "layers enabled: ";
  //   for(auto &layer : layers_enabled)
  //     std::cout << layer << " ";
  //   std::cout << std::endl;
  // }
};
#pragma pack(pop, sp)
