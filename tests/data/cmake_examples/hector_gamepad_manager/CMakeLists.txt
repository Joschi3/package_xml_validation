cmake_minimum_required(VERSION 3.8)
project(hector_gamepad_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

option(ENABLE_COVERAGE "Enable coverage flags" OFF)
if(ENABLE_COVERAGE AND (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang"))
  add_compile_options(-O0 -g --coverage)
  add_link_options(--coverage)
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
install(DIRECTORY test/config/ DESTINATION share/${PROJECT_NAME}/test/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(rtest REQUIRED)
  find_package(GTest CONFIG REQUIRED COMPONENTS GTest GMock)
  find_package(controller_manager_msgs REQUIRED)

  # 1. Generate Test Doubles
  test_tools_add_doubles(${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_test_doubles ${DEPENDENCIES})

  # 1. Create the GTest/GMock executable
  ament_add_gmock(
    ${PROJECT_NAME}_test test/main.cpp test/test_hector_gamepad_manager.cpp
    test/test_hector_gamepad_manager_internal.cpp)

  # 1. Link against the Test Doubles
  target_link_libraries(${PROJECT_NAME}_test
                        ${PROJECT_NAME}::${PROJECT_NAME}_test_doubles)
  ament_target_dependencies(${PROJECT_NAME}_test ${DEPENDENCIES} rtest
                            controller_manager_msgs)

  # 1. Add includes for tests
  target_include_directories(${PROJECT_NAME}_test
                             PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

  set(PLUGIN_TEST_PREFIX
      "${CMAKE_BINARY_DIR}/../hector_gamepad_manager_plugins")
  set_tests_properties(
    ${PROJECT_NAME}_test
    PROPERTIES
      ENVIRONMENT
      "AMENT_PREFIX_PATH=${PLUGIN_TEST_PREFIX}:${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}"
  )
endif()

ament_export_targets(${PROJECT_NAME}-targets)
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
