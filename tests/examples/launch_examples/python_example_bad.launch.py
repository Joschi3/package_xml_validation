# from https://docs.ros.org/en/rolling/How-To-Guides/Launch-file-different-formats.html
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    my_package = "demo_nodes_cpp"
    turtle_pkg = "turtlesim"
    package = "something_else"  # Placeholder for the package name
    launch_dir = PathJoinSubstitution(
        [FindPackageShare(my_package), "launch", "topics"]
    )
    return LaunchDescription(
        [
            # args that can be set from the command line or a default will be used
            DeclareLaunchArgument("background_r", default_value="0"),
            DeclareLaunchArgument("background_g", default_value="255"),
            DeclareLaunchArgument("background_b", default_value="0"),
            DeclareLaunchArgument("chatter_py_ns", default_value="chatter/py/ns"),
            DeclareLaunchArgument("chatter_xml_ns", default_value="chatter/xml/ns"),
            DeclareLaunchArgument("chatter_yaml_ns", default_value="chatter/yaml/ns"),
            # include another launch file
            IncludeLaunchDescription(
                PathJoinSubstitution([launch_dir, "talker_listener_launch.py"])
            ),
            # include a Python launch file in the chatter_py_ns namespace
            GroupAction(
                actions=[
                    # push_ros_namespace first to set namespace of included nodes for following actions
                    PushROSNamespace("chatter_py_ns"),
                    IncludeLaunchDescription(
                        PathJoinSubstitution([launch_dir, "talker_listener_launch.py"])
                    ),
                ]
            ),
            # include a xml launch file in the chatter_xml_ns namespace
            GroupAction(
                actions=[
                    # push_ros_namespace first to set namespace of included nodes for following actions
                    PushROSNamespace("chatter_xml_ns"),
                    IncludeLaunchDescription(
                        PathJoinSubstitution([launch_dir, "talker_listener_launch.xml"])
                    ),
                ]
            ),
            # include a yaml launch file in the chatter_yaml_ns namespace
            GroupAction(
                actions=[
                    # push_ros_namespace first to set namespace of included nodes for following actions
                    PushROSNamespace("chatter_yaml_ns"),
                    IncludeLaunchDescription(
                        PathJoinSubstitution(
                            [launch_dir, "talker_listener_launch.yaml"]
                        )
                    ),
                ]
            ),
            # start a turtlesim_node in the turtlesim1 namespace
            Node(
                package=turtle_pkg,
                namespace="turtlesim1",
                executable="turtlesim_node",
                name="sim",
            ),
            # start another turtlesim_node in the turtlesim2 namespace
            # and use args to set parameters
            Node(
                package=turtle_pkg,
                namespace="turtlesim2",
                executable="turtlesim_node",
                name="sim",
                parameters=[
                    {
                        "background_r": LaunchConfiguration("background_r"),
                        "background_g": LaunchConfiguration("background_g"),
                        "background_b": LaunchConfiguration("background_b"),
                    }
                ],
            ),
            # perform remap so both turtles listen to the same command topic
            Node(
                package=turtle_pkg,
                executable="mimic",
                name="mimic",
                remappings=[
                    ("/input/pose", "/turtlesim1/turtle1/pose"),
                    ("/output/cmd_vel", "/turtlesim2/turtle1/cmd_vel"),
                ],
            ),
        ]
    )
