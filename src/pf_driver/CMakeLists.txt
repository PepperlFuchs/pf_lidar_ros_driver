cmake_minimum_required(VERSION 3.7)
project(pf_driver)

find_package(ament_cmake REQUIRED)

find_package(pf_interfaces)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pf_interfaces REQUIRED)

find_package(Boost REQUIRED COMPONENTS  thread system)
if(NOT TARGET JsonCpp::JsonCpp)
  find_package(jsoncpp REQUIRED)
elseif(NOT TARGET jsoncpp_lib)
  add_library(jsoncpp_lib ALIAS JsonCpp::JsonCpp)
endif()
find_package(yaml-cpp REQUIRED)
get_target_property(jsoncpp_INCLUDE_DIRS jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CURLPP REQUIRED curlpp)

include_directories(
  include
  ${jsoncpp_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_SOURCES
  src/pf/pf_interface.cpp
  src/pf/parser_utils.cpp
  src/pf/pf_packet_reader.cpp
  src/pf/pfsdp_base.cpp
  src/pf/pipeline.cpp
  src/pf/http_helpers/curl_resource.cpp
  src/pf/http_helpers/http_helpers.cpp
  src/pf/http_helpers/http_interface.cpp
  src/pf/pf_packet/pf_packet.cpp
  src/pf/pf_packet/pf_r2000_packet.cpp
  src/pf/pf_packet/pf_r2000_packet_a.cpp
  src/pf/pf_packet/pf_r2000_packet_b.cpp
  src/pf/pf_packet/pf_r2000_packet_c.cpp
  src/pf/pf_packet/pf_r2300_packet.cpp
  src/pf/pf_packet/pf_r2300_packet_c1.cpp
  src/pf/r2000/pfsdp_2000.cpp
  src/pf/r2300/pfsdp_2300.cpp
  src/communication/udp_transport.cpp
  src/communication/tcp_transport.cpp
  src/ros/laser_scan_publisher.cpp
  src/ros/point_cloud_publisher.cpp
  src/ros/pf_data_publisher.cpp
)

add_executable(ros_main
  ${${PROJECT_NAME}_SOURCES}
  src/ros/ros_main.cpp
)
target_link_libraries(ros_main
  curlpp
  curl
  jsoncpp
  yaml-cpp
)
ament_target_dependencies(ros_main
  rclcpp
  pf_interfaces
  laser_geometry
  tf2_ros
  pcl_ros
  pf_interfaces
)

#roslint_cpp(src/ros/ros_main.cpp
#            src/ros/scan_publisher.cpp
#            src/pf/pf_interface.cpp
#            src/pf/pf_packet.cpp
#            src/communication.cpp)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(${PROJECT_NAME}_TEST_SOURCES
  #  tests/http_request.cpp
  #  tests/pfsdp_protocol.cpp)
    tests/pipeline.cpp
    tests/parse_packet.cpp
  )

  # Add gtests
  ament_add_gtest(${PROJECT_NAME}_test
    ${${PROJECT_NAME}_SOURCES}
    ${${PROJECT_NAME}_TEST_SOURCES}
    tests/main.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_test
    curlpp
    curl
    jsoncpp
    yaml-cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
    laser_geometry
    pcl_ros
    pf_interfaces
    rclcpp
  )
endif()

install(TARGETS ros_main
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config dumps
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include)

ament_export_include_directories(include)

ament_package()
