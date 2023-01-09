cmake_minimum_required(VERSION 3.7)
project(pf_driver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  laser_geometry
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  roscpp_serialization
  roslint
  sensor_msgs
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS  thread system)
find_package(jsoncpp REQUIRED)
get_target_property(jsoncpp_INCLUDE_DIRS jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CURLPP REQUIRED curlpp)

generate_dynamic_reconfigure_options(
  cfg/PFDriverR2000.cfg
  cfg/PFDriverR2300.cfg
)

add_message_files(
  FILES
  PFHeader.msg
  PFR2000Header.msg
  PFR2300Header.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp roscpp_serialization sensor_msgs std_msgs tf
)

include_directories(
  include
  ${jsoncpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_main src/pf/pf_interface.cpp
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
                        src/ros/ros_main.cpp
                        src/ros/laser_scan_publisher.cpp
                        src/ros/point_cloud_publisher.cpp
                        src/ros/pf_data_publisher.cpp
                        )
target_link_libraries(ros_main
  ${catkin_LIBRARIES} curlpp curl jsoncpp
)
add_dependencies(ros_main ${PROJECT_NAME}_gencfg)
add_dependencies(ros_main ${${PROJECT_NAME}_EXPORTED_TARGETS})

roslint_cpp(src/ros/ros_main.cpp
            src/ros/scan_publisher.cpp
            src/pf/pf_interface.cpp
            src/pf/pf_packet.cpp
            src/communication.cpp)

#if (CATKIN_ENABLE_TESTING)
#  set(${PROJECT_NAME}_TEST_SOURCES
#    tests/http_request.cpp
#    tests/pfsdp_protocol.cpp)

#  catkin_add_gtest(pf_driver_test ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_TEST_SOURCES} tests/main.cpp)
#  target_link_libraries(pf_driver_test ${catkin_LIBRARIES} cpprestsdk::cpprest)
#endif()

install(TARGETS ros_main ros_main
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
