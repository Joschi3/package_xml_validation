import rosdep2
from rosdep2.lookup import RosdepLookup
from rosdep2.rospkg_loader import DEFAULT_VIEW_KEY
from rosdep2.sources_list import SourcesListLoader
from rosdep2.installers import PackageManagerInstaller
from rospkg import RosPack, os_detect


def check_rosdeps(dependencies) -> list[str]:
    """
    Check if rosdep can resolve all specified ROS dependencies.

    :param dependencies: List of ROS dependency names to check.
    :return: List of dependencies that rosdep cannot resolve.
    """

    rosdep_installer_context = rosdep2.create_default_installer_context()
    rosdep = rosdep2.RosdepLookup.create_from_rospkg()
    (
        rosdep_os_name,
        rosdep_os_version,
    ) = rosdep_installer_context.get_os_name_and_version()
    rosdep_view = rosdep.get_rosdep_view(rosdep2.rospkg_loader.DEFAULT_VIEW_KEY)

    unresolvable = []
    for dep in dependencies:
        try:
            # Resolve the dependency
            dep: rosdep2.RosdepDefinition = rosdep_view.lookup(dep)
            installer, data = dep.get_rule_for_platform(
                rosdep_os_name, rosdep_os_version, ["apt", "pip"], "apt"
            )
            if installer is None:
                unresolvable.append(dep)
        except:
            unresolvable.append(dep)

    return unresolvable


if __name__ == "__main__":
    dependencies = ["rclcpp", "nonexistent_dependency", "hector_gamepad_manager"]
    unresolvable = check_rosdeps(dependencies)
    for dep in unresolvable:
        print(f"Could not resolve dependency: {dep}")
