cmake_minimum_required(VERSION 3.8)
project(pf_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PFHeader.msg"
  "msg/PFR2000Header.msg"
  "msg/PFR2300Header.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
