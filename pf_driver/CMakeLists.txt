cmake_minimum_required(VERSION 3.7)
project(pf_driver)

add_compile_options(-std=c++14)

## Currently a bug with cpprestsdk
## https://github.com/microsoft/cpprestsdk/issues/686#issuecomment-440622042
set(cpprestsdk_DIR /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  dynamic_reconfigure
  laser_geometry
  tf
  pcl_conversions
  pcl_ros
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(cpprestsdk REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/settings.cfg
)



catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
)

add_executable(r2000_node src/r2000_node.cpp)

target_link_libraries(r2000_node
  ${catkin_LIBRARIES} cpprestsdk::cpprest
)

add_dependencies(r2000_node ${PROJECT_NAME}_gencfg)

add_executable(r2300_node src/r2300_node.cpp)

target_link_libraries(r2300_node
  ${catkin_LIBRARIES} cpprestsdk::cpprest
)

add_dependencies(r2300_node ${PROJECT_NAME}_gencfg)

