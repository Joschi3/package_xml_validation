#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # === Declare launch arguments ===
    gui_arg = DeclareLaunchArgument(
        name="gui", default_value="true", description="Whether to launch gzclient"
    )

    log_level_arg = DeclareLaunchArgument(
        name="log_level",
        default_value="2",
        description="Gazebo log level: 0=quiet, 1=error, 2=warn, 3=info, 4=debug",
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty.sdf",
        description="Name of the world to load",
    )

    # === Load configurations ===
    gui = LaunchConfiguration("gui")
    log_level = LaunchConfiguration("log_level")
    world_name = LaunchConfiguration("world_name")

    world = PathJoinSubstitution(
        [FindPackageShare("gazebo_robot_sim_athena"), "worlds", world_name]
    )

    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    global_bridge_params = os.path.join(
        get_package_share_directory("gazebo_robot_sim_athena"),
        "config",
        "global_gazebo_bridge_params.yaml",
    )

    # === gzserver (headless physics engine) ===
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                TextSubstitution(text="-s -r -v"),
                log_level,
                TextSubstitution(text=" "),
                world,
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # === gzclient (conditionally started GUI) ===
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [TextSubstitution(text="-g -v"), log_level]
        }.items(),
        condition=IfCondition(gui),
    )

    # === Bridge global topics ===
    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"config_file": global_bridge_params},
            {"expand_gz_topic_names": True},
            {"namespace": "athena"},
        ],
        arguments=[
            "--ros-args",
            # '-p',
            # f'config_file:={bridge_params}',
        ],
        output="screen",
    )

    # === Assemble LaunchDescription ===
    ld = LaunchDescription()
    ld.add_action(gui_arg)
    ld.add_action(log_level_arg)
    ld.add_action(world_name_arg)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)

    return ld
