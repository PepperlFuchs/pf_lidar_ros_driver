cmake_minimum_required(VERSION 3.7)
project(pf_driver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  dynamic_reconfigure
  laser_geometry
  tf
  pcl_conversions
  pcl_ros
  roscpp_serialization
)

find_package(Boost REQUIRED COMPONENTS  thread system)
find_package(jsoncpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CURLPP REQUIRED curlpp)

generate_dynamic_reconfigure_options(
  cfg/PFDriver.cfg
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
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime roscpp_serialization
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_main src/ros_main.cpp
                        src/pf_interface.cpp
                        src/scan_publisher.cpp
                        src/pf_packet.cpp
                        src/communication.cpp)
target_link_libraries(ros_main
  ${catkin_LIBRARIES} curlpp curl jsoncpp
)
add_dependencies(ros_main ${PROJECT_NAME}_gencfg)

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

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
