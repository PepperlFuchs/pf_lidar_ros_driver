cmake_minimum_required(VERSION 3.7)
project(pf_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(tests/test_r2000_bringup.launch.py)
  add_launch_test(tests/test_r2300_bringup.launch.py)
endif()

install(DIRECTORY launch meshes rviz urdf DESTINATION share/${PROJECT_NAME})

ament_package()
