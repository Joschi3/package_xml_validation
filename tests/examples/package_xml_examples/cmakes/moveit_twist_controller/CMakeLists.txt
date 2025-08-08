cmake_minimum_required(VERSION 3.16)
project(moveit_twist_controller)

################################################################################
# Compiler warnings and Windows symbol exports
################################################################################
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(
            -Wall
            -Wextra
            -Wpedantic
            -Werror=conversion
            -Werror=unused-but-set-variable
            -Werror=return-type
            -Werror=shadow
            -Werror=format
            -Werror=range-loop-construct
            -Werror=missing-braces
    )
endif()

# Export all symbols on Windows. This avoids needing explicit visibility macros.
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

################################################################################
# Dependencies
################################################################################
set(THIS_PACKAGE_INCLUDE_DEPENDS
        moveit_core
        moveit_ros_planning
        sensor_msgs
        urdf
        moveit_msgs
        tf2_ros
        controller_interface
        hardware_interface
        std_msgs
        std_srvs
        pluginlib
        rclcpp
        rclcpp_lifecycle
        generate_parameter_library
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

################################################################################
# Library
################################################################################
include_directories(include)

generate_parameter_library(moveit_twist_controller_parameters
        src/twist_controller.yaml
)

add_library(moveit_twist_controller SHARED
        src/twist_controller.cpp
        src/inverse_kinematics.cpp
        src/common.cpp
)

target_compile_features(moveit_twist_controller PUBLIC cxx_std_17)
target_include_directories(moveit_twist_controller PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/moveit_twist_controller>
)
target_link_libraries(moveit_twist_controller PUBLIC
        moveit_twist_controller_parameters
)
ament_target_dependencies(moveit_twist_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(controller_interface moveit_twist_controller.xml)
################################################################################
# Testing
################################################################################
if(BUILD_TESTING)
    find_package(ament_cmake_gmock REQUIRED)
    find_package(controller_manager REQUIRED)
    find_package(hardware_interface REQUIRED)
    find_package(ros2_control_test_assets REQUIRED)

    add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")

    ament_add_gmock(
            test_load_moveit_twist_controller
            test/test_load_moveit_twist_controller.cpp
    )
    ament_target_dependencies(
            test_load_moveit_twist_controller
            controller_manager
            hardware_interface
            ros2_control_test_assets
    )

    # If you have additional tests, add them similarly.
    # ament_add_gmock(...)
endif()

################################################################################
# Installation
################################################################################
install(
        DIRECTORY include/
        DESTINATION include/moveit_twist_controller
)
install(
        TARGETS
        moveit_twist_controller
        moveit_twist_controller_parameters
        EXPORT export_moveit_twist_controller
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
)

install(
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

#[[install(
        FILES moveit_twist_controller.xml
        DESTINATION share/${PROJECT_NAME}
)]]

################################################################################
# Export
################################################################################
ament_export_targets(export_moveit_twist_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
