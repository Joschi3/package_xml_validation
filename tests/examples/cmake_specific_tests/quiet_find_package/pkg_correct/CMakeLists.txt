cmake_minimum_required(VERSION 3.8)
project(quiet_find_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(benchmark QUIET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()
