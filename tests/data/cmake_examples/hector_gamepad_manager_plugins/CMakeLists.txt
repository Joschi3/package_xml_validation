cmake_minimum_required(VERSION 3.16)
project(hector_gamepad_manager_plugins LANGUAGES CXX)

option(ENABLE_COVERAGE "Enable coverage flags" OFF)
if(ENABLE_COVERAGE AND (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang"))
  add_compile_options(-O0 -g --coverage)
  add_link_options(--coverage)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gen_version_h REQUIRED)
find_package(Eigen3 REQUIRED)

# Define a list of dependencies in alphabetical order
set(DEPENDENCIES
    controller_manager_msgs
    controller_orchestrator
    geometry_msgs
    hector_gamepad_plugin_interface
    hector_ros2_utils
    moveit_msgs
    pluginlib
    ros_babel_fish
    rcl_interfaces
    rclcpp
    rclcpp_action
    sensor_msgs
    srdfdom
    std_srvs)

foreach(dependency IN LISTS DEPENDENCIES)
  find_package(${dependency} REQUIRED)
endforeach()
find_package(backward_ros REQUIRED)
set(HEADERS
    include/hector_gamepad_manager_plugins/controller_helper.hpp
    include/hector_gamepad_manager_plugins/drive_plugin.hpp
    include/hector_gamepad_manager_plugins/battery_monitor_plugin.hpp
    include/hector_gamepad_manager_plugins/flipper_plugin.hpp
    include/hector_gamepad_manager_plugins/manipulation_plugin.hpp
    include/hector_gamepad_manager_plugins/moveit_plugin.hpp
    include/hector_gamepad_manager_plugins/blackboard_plugin.hpp
    include/hector_gamepad_manager_plugins/test_probe_plugin.hpp)

set(SOURCES
    src/controller_helper.cpp
    src/drive_plugin.cpp
    src/flipper_plugin.cpp
    src/battery_monitor_plugin.cpp
    src/manipulation_plugin.cpp
    src/moveit_plugin.cpp
    src/blackboard_plugin.cpp
    src/test_probe_plugin.cpp)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

if(BUILD_TESTING)
  # Here we create a library with test doubles to be used in tests rtest
  # provides mocks for rclcpp entities like publishers, subscriptions, services,
  # and service clients. This allows us to test our code without a running ROS2
  # graph.
  find_package(ament_cmake_gmock REQUIRED)
  find_package(GTest CONFIG REQUIRED COMPONENTS GTest GMock)
  find_package(rtest REQUIRED)
  add_library(${PROJECT_NAME}_test_doubles SHARED ${SOURCES} ${HEADERS})
  target_include_directories(
    ${PROJECT_NAME}_test_doubles
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_doubles PUBLIC ${DEPENDENCIES})
  target_link_libraries(
    ${PROJECT_NAME}_test_doubles
    PRIVATE rtest::publisher_mock rtest::subscription_mock rtest::service_mock
            rtest::service_client_mock rtest::rtest_common)
  set_target_properties(
    ${PROJECT_NAME}_test_doubles
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib"
               ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib"
               RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")

  file(MAKE_DIRECTORY
       "${CMAKE_CURRENT_BINARY_DIR}/share/hector_gamepad_manager_plugins")
  file(MAKE_DIRECTORY
       "${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/"
       "hector_gamepad_manager__pluginlib__plugin")
  file(MAKE_DIRECTORY
       "${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages")
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins_test.xml
    ${CMAKE_CURRENT_BINARY_DIR}/share/hector_gamepad_manager_plugins/plugins_test.xml
    COPYONLY)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    ${CMAKE_CURRENT_BINARY_DIR}/share/hector_gamepad_manager_plugins/package.xml
    COPYONLY)
  file(
    WRITE
    "${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/hector_gamepad_manager__pluginlib__plugin/hector_gamepad_manager_plugins"
    "share/hector_gamepad_manager_plugins/plugins_test.xml\n")
  file(
    WRITE
    "${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/hector_gamepad_manager_plugins"
    "${CMAKE_CURRENT_BINARY_DIR}\n")
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

# Install the library to the plugins directory
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

# Install the plugin XML
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(hector_gamepad_manager plugins.xml)

ament_export_targets(${PROJECT_NAME}-targets)
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
