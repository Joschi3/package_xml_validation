from ament_index_python.packages import get_package_prefix, get_package_share_path
from launch import LaunchDescription
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    prefix_a = FindPackagePrefix("foo_pkg")
    prefix_b = FindPackagePrefix(package="bar_pkg")
    share_path = get_package_share_path("baz_pkg")
    install_prefix = get_package_prefix("qux_pkg")

    _ = (prefix_a, prefix_b, share_path, install_prefix)
    return LaunchDescription([])
