cmake_minimum_required(VERSION 3.8)
project(hector_gamepad_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
set(DEPENDENCIES pluginlib rclcpp sensor_msgs yaml-cpp
                 hector_gamepad_plugin_interface)

foreach(dependency IN LISTS DEPENDENCIES)
  find_package(${dependency} REQUIRED)
endforeach()

set(HEADERS include/hector_gamepad_manager/hector_gamepad_manager.hpp)

set(SOURCES src/hector_gamepad_manager.cpp)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

add_executable(hector_gamepad_manager_node src/hector_gamepad_manager_node.cpp)
target_link_libraries(hector_gamepad_manager_node PUBLIC ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS hector_gamepad_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  ament_add_gtest_executable(${PROJECT_NAME}_test
                             test/test_hector_gamepad_manager.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_test
                            ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_ros_test(test/test_hector_gamepad_manager.launch.py ARGS
               "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()

ament_export_targets(${PROJECT_NAME}-targets)
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
