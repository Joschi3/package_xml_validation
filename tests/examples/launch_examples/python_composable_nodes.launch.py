from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription(
        [
            LoadComposableNodes(
                target_container="my_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="image_proc",
                        plugin="image_proc::RectifyNode",
                        name="rectify",
                    ),
                ],
            ),
            LifecycleNode(
                package="lifecycle_demo",
                executable="lifecycle_talker",
                name="talker",
                namespace="",
            ),
        ]
    )
