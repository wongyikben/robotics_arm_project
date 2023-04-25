cmake_minimum_required(VERSION 3.5)
# Template taken from https://github.com/ros2/tutorials/blob/master/rclcpp_tutorials/CMakeLists.txt
project(cr_description)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes xacro rviz ros2_control urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()