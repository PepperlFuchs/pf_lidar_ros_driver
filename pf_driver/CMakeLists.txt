cmake_minimum_required(VERSION 3.7)
project(pf_driver)

add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)

find_package(pf_interfaces)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Boost REQUIRED COMPONENTS  thread system)
find_package(jsoncpp REQUIRED)
get_target_property(jsoncpp_INCLUDE_DIRS jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CURLPP REQUIRED curlpp)

include_directories(
  include
  ${jsoncpp_INCLUDE_DIRS}
)

add_executable(ros_main src/communication.cpp
                        src/pf/pf_interface.cpp
                        src/pf/pf_packet.cpp
                        src/pf/pfsdp_protocol.cpp
                        src/ros/ros_main.cpp
                        src/ros/scan_publisher.cpp
                        src/ros/hmi_image_listener.cpp
                        )
target_link_libraries(ros_main
  curlpp
  curl
  jsoncpp
  b64
)
ament_target_dependencies(ros_main
  rclcpp
  pf_interfaces
  laser_geometry
  tf2_ros
  pcl_ros
)

#roslint_cpp(src/ros/ros_main.cpp
#            src/ros/scan_publisher.cpp
#            src/pf/pf_interface.cpp
#            src/pf/pf_packet.cpp
#            src/communication.cpp)

#if (CATKIN_ENABLE_TESTING)
#  set(${PROJECT_NAME}_TEST_SOURCES
#    tests/http_request.cpp
#    tests/pfsdp_protocol.cpp)

#  catkin_add_gtest(pf_driver_test ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_TEST_SOURCES} tests/main.cpp)
#  target_link_libraries(pf_driver_test ${catkin_LIBRARIES} cpprestsdk::cpprest)
#endif()

install(TARGETS ros_main
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include)

ament_export_include_directories(include)

ament_package()
