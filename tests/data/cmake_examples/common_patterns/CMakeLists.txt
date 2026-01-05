cmake_minimum_required(VERSION 3.5)
project(common_patterns)

find_package(ament_cmake REQUIRED)
find_package(Boost 1.74 REQUIRED COMPONENTS filesystem system)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Eigen3 3.3 REQUIRED)
find_package(
  yaml-cpp
  REQUIRED
)
find_package(Python3 3.10 QUIET)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

set(LOCAL_DEPS fmt spdlog)
foreach(dep IN LISTS LOCAL_DEPS)
  find_package(${dep} REQUIRED)
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(benchmark REQUIRED)
endif()
