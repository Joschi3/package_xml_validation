import os
import unittest

from package_xml_validation.helpers.find_launch_dependencies import scan_file


class TestFindLaunchDependencies(unittest.TestCase):
    # ─── Define test cases: filename → expected packages ───
    EXAMPLES = {
        "python_example.launch.py": ["demo_nodes_cpp", "turtlesim"],
        "xml_example.launch.xml": ["demo_nodes_cpp", "turtlesim"],
        "yaml_example.launch.yml": ["demo_nodes_cpp", "turtlesim"],
        "hector_launch_component.yaml": ["athena_announcer"],
        "python_world.launch.py": [
            "gazebo_robot_sim_athena",
            "ros_gz_sim",
            "ros_gz_bridge",
        ],
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


if __name__ == "__main__":
    unittest.main()
