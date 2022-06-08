cmake_minimum_required(VERSION 3.7)
project(pf_description)

find_package(ament_cmake REQUIRED)

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest(tests/test_r2000_bringup.launch)
#  add_rostest(tests/test_r2300_bringup.launch)
#endif()

install(DIRECTORY launch urdf meshes
        DESTINATION share/${PROJECT_NAME})

ament_package()
