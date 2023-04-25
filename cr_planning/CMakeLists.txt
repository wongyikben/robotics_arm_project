cmake_minimum_required(VERSION 3.5)
# Template taken from https://github.com/ros2/tutorials/blob/master/rclcpp_tutorials/CMakeLists.txt
project(cr_planning)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# https://stackoverflow.com/questions/12399422/how-to-set-linker-flags-for-openmp-in-cmakes-try-compile-function


include_directories(${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(pcl_filter_lib SHARED src/pcl_filter.cpp)

ament_target_dependencies(pcl_filter_lib
  rclcpp
  PCL
  pcl_conversions
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
)

add_executable(pcl_filter src/filter_main.cpp)
target_link_libraries(pcl_filter ${PCL_LIBRARIES} pcl_filter_lib)

install(TARGETS
  pcl_filter
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  cr_planning/moveit_demo_client.py
  cr_planning/pcl_obstacle_moveit_client.py
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_include_directories(include)
ament_package()
