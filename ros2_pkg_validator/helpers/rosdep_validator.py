import rosdep2


class RosdepValidator:
    """
    Class to validate ROS dependencies using rosdep.
    """

    def __init__(self):
        self.rosdep_installer_context = rosdep2.create_default_installer_context()
        self.rosdep = rosdep2.RosdepLookup.create_from_rospkg()
        (
            self.rosdep_os_name,
            self.rosdep_os_version,
        ) = self.rosdep_installer_context.get_os_name_and_version()
        self.rosdep_view = self.rosdep.get_rosdep_view(
            rosdep2.rospkg_loader.DEFAULT_VIEW_KEY
        )

    def check_rosdeps(self, dependencies) -> list[str]:
        """
        Check if rosdep can resolve all specified ROS dependencies.

        :param dependencies: List of ROS dependency names to check.
        :return: List of dependencies that rosdep cannot resolve.
        """

        unresolvable = []
        for dep in dependencies:
            try:
                # Resolve the dependency
                dep: rosdep2.RosdepDefinition = self.rosdep_view.lookup(dep)
                installer, data = dep.get_rule_for_platform(
                    self.rosdep_os_name, self.rosdep_os_version, ["apt", "pip"], "apt"
                )
                if installer is None:
                    unresolvable.append(dep)
            except:
                unresolvable.append(dep)

        return unresolvable


if __name__ == "__main__":
    dependencies = ["rclcpp", "nonexistent_dependency", "hector_gamepad_manager"]
    validator = RosdepValidator()
    # Example usage
    unresolvable = validator.check_rosdeps(dependencies)
    for dep in unresolvable:
        print(f"Could not resolve dependency: {dep}")
