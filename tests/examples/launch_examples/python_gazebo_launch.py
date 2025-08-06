from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import AppendEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
import os


def generate_launch_description():
    # Use simulation time
    set_use_sim_time = SetParameter(name="use_sim_time", value=True)

    # Set the path to the Gazebo ROS package
    ros_gz_sim = FindPackageShare("ros_gz_sim")

    # Set the path to map package.
    this_pkg = FindPackageShare(package="simulation_scenario_robocup_gazebo")
    # Set the path to the SDF model files.
    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(
            get_package_share_directory("simulation_scenario_robocup_gazebo"),
            "nist_arena_models",
        ),
    )

    # Set the default path to the world file
    map_id = LaunchConfiguration("map_id", default="exp1")
    map_path = PathJoinSubstitution(
        [
            this_pkg,
            "worlds",
            LaunchConfiguration("map_file", default=[map_id, ".world"]),
        ]
    )

    # Load parameter bridge config
    global_bridge_params = os.path.join(
        get_package_share_directory("simulation_scenario_robocup_gazebo"),
        "config",
        "global_gazebo_bridge_params.yaml",
    )

    # Specify the actions
    declare_map_cmd = DeclareLaunchArgument(
        name="map_id", default_value="exp1", description="Map to load"
    )

    # conditional gui and parametrized log level
    gui = LaunchConfiguration("gui")
    log_level = LaunchConfiguration("log_level")

    gui_arg = DeclareLaunchArgument(
        name="gui", default_value="true", description="Whether to launch gzclient"
    )

    log_level_arg = DeclareLaunchArgument(
        name="log_level",
        default_value="2",
        description="Gazebo log level: 0=quiet, 1=error, 2=warn, 3=info, 4=debug",
    )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        # launch_arguments={'gz_args':['-u --verbose ', map_path],
        #                  'on_exit_shutdown': 'true'}.items()
        launch_arguments={
            "gz_args": ["-r -s -v", log_level, " ", map_path],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [TextSubstitution(text="-g -v"), log_level]
        }.items(),
        condition=IfCondition(gui),
    )

    # Bridge clock and other global unique topics here
    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"config_file": global_bridge_params},
            {"expand_gz_topic_names": True},
        ],
        arguments=[
            "--ros-args",
        ],
        output="screen",
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(gui_arg)
    ld.add_action(log_level_arg)
    ld.add_action(set_use_sim_time)
    ld.add_action(set_env_vars_resources)
    # Declare the launch options
    ld.add_action(declare_map_cmd)
    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(gzclient_cmd)
    # Bridging gz topics to ros
    ld.add_action(start_gazebo_ros_bridge_cmd)

    return ld
