cmake_minimum_required(VERSION 3.10)
project(hector_ros2_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(hector_ros2_utils INTERFACE)
add_library(hector_ros2_utils::hector_ros2_utils ALIAS hector_ros2_utils)
target_include_directories(hector_ros2_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hector_ros2_utils INTERFACE rclcpp::rclcpp)

#############
## Install ##
#############

install(TARGETS hector_ros2_utils EXPORT hector_ros2_utils-targets)
install(
  EXPORT hector_ros2_utils-targets
  FILE hector_ros2_utils-targets.cmake
  NAMESPACE hector_ros2_utils::
  DESTINATION lib/cmake/hector_ros2_utils
)

install(DIRECTORY include/ DESTINATION include/)


#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
#  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()

  ament_add_gtest_executable(utils test/utils.cpp)
  target_include_directories(utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(utils hector_ros2_utils)

  ament_add_gtest_executable(test_parameters test/test_parameters.cpp)
  target_include_directories(test_parameters PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(test_parameters hector_ros2_utils)
endif()

ament_export_targets(hector_ros2_utils-targets)
ament_export_dependencies(rclcpp)
ament_export_include_directories(include)
ament_package()
