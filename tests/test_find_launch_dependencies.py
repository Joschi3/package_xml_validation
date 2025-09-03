import os
import unittest

from package_xml_validation.helpers.find_launch_dependencies import (
    scan_file,
    scan_files,
)


class TestFindLaunchDependencies(unittest.TestCase):
    # ─── Define test cases: filename → expected packages ───
    EXAMPLES = {
        "python_example.launch.py": ["demo_nodes_cpp", "turtlesim"],
        "python_example_comment.launch.py": [
            "demo_nodes_cpp",
            "turtlesim",
        ],  # make sure to ignore commented pkgs
        "xml_example.launch.xml": ["demo_nodes_cpp", "turtlesim"],
        "xml_example_comment.launch.xml": ["demo_nodes_cpp", "turtlesim"],
        "yaml_example.launch.yml": ["demo_nodes_cpp", "turtlesim"],
        "yaml_example_comment.launch.yml": ["demo_nodes_cpp", "turtlesim"],
        "hector_launch_component.yaml": ["athena_announcer"],
        "hector_launch_component_comment.yaml": ["athena_announcer"],
        "python_world.launch.py": [
            "gazebo_robot_sim_athena",
            "ros_gz_sim",
            "ros_gz_bridge",
        ],
        "python_gazebo_launch.py": [
            "simulation_scenario_robocup_gazebo",
            "ros_gz_sim",
            "ros_gz_bridge",
        ],
        "python_example_bad.launch.py": [],  # ["demo_nodes_cpp", "turtlesim"],
        "yaml_example_bad.launch.yml": [],  # ["demo_nodes_cpp", "turtlesim"],
    }

    # Directory where example launch files live
    BASE_DIR = os.path.join(os.path.dirname(__file__), "examples", "launch_examples")

    def test_scan_each_file(self):
        """Ensure scan_file finds exactly the expected packages in each example."""
        for filename, expected_pkgs in self.EXAMPLES.items():
            with self.subTest(filename=filename):
                path = os.path.join(self.BASE_DIR, filename)
                # 1) file must exist
                self.assertTrue(os.path.isfile(path), f"Example file not found: {path}")
                # 2) scan it
                found = set()
                print(f"Scanning {path} for launch dependencies...")
                scan_file(path, found)
                # 3) compare sets
                self.assertEqual(
                    set(expected_pkgs),
                    found,
                    msg=(
                        f"For '{filename}':\n"
                        f"  expected: {sorted(expected_pkgs)}\n"
                        f"  found:    {sorted(found)}"
                    ),
                )

    def test_scan_each_file_given_file(self):
        """When given a file path, or an not existing directory scan_files should return an empty list"""
        found = scan_files("non_existing_file.launch.py")
        self.assertEqual(len(found), 0)
        existing_file = os.path.join(
            os.path.dirname(__file__),
            "examples",
            "launch_examples",
            "python_example.launch.py",
        )
        found = scan_files(existing_file)
        self.assertEqual(len(found), 0)


if __name__ == "__main__":
    unittest.main()
