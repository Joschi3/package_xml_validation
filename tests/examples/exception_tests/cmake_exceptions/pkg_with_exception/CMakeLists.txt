cmake_minimum_required(VERSION 3.5)
project(test_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)

ament_package()
