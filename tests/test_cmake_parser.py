import os
import unittest

from package_xml_validation.helpers.cmake_parsers import read_deps_from_cmake_file

# This test suite validates the CMake parser against known CMakeLists.txt examples.
# The examples are stored in the 'data/cmake_examples' directory.
# Each example has expected main and test dependencies defined in the 'cases' dictionary.


class TestCMakeParser(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        current_dir = os.path.dirname(__file__)
        cls.examples_dir = os.path.join(current_dir, "data", "cmake_examples")
        cls.cases = {
            "hector_gamepad_manager_plugins": {
                "main": [
                    "ament_cmake_gen_version_h",
                    "Eigen3",
                    "controller_manager_msgs",
                    "controller_orchestrator",
                    "geometry_msgs",
                    "hector_gamepad_plugin_interface",
                    "hector_ros2_utils",
                    "moveit_msgs",
                    "pluginlib",
                    "ros_babel_fish",
                    "rcl_interfaces",
                    "rclcpp",
                    "rclcpp_action",
                    "sensor_msgs",
                    "srdfdom",
                    "std_srvs",
                    "backward_ros",
                ],
                "test": ["ament_cmake_gmock", "GTest", "rtest"],
            },
            "hector_gamepad_manager": {
                "main": [
                    "pluginlib",
                    "rclcpp",
                    "sensor_msgs",
                    "yaml-cpp",
                    "hector_gamepad_plugin_interface",
                ],
                "test": [
                    "ament_cmake_gmock",
                    "rtest",
                    "GTest",
                    "controller_manager_msgs",
                ],
            },
            "hector_ros2_utils": {
                "main": ["rclcpp"],
                "test": ["ament_cmake_gtest"],
            },
            "common_patterns": {
                "main": [
                    "Boost",
                    "Qt5",
                    "Eigen3",
                    "yaml-cpp",
                    "Python3",
                    "fmt",
                    "spdlog",
                ],
                "test": ["ament_cmake_gtest", "benchmark"],
                "absent": ["OpenMP", "Threads", "ament_cmake"],
            },
        }

    def test_parse_cmake_examples(self):
        for case_name, expected in self.cases.items():
            cmake_file = os.path.join(self.examples_dir, case_name, "CMakeLists.txt")
            with self.subTest(case=case_name):
                self.assertTrue(
                    os.path.exists(cmake_file),
                    f"Missing example file: {cmake_file}",
                )
                main_deps, test_deps = read_deps_from_cmake_file(cmake_file)
                print(f"Case: {case_name}")
                print(f"  Main deps: {main_deps}")
                print(f"  Test deps: {test_deps}")
                self.assertCountEqual(main_deps, expected["main"])
                self.assertCountEqual(test_deps, expected["test"])

                for dep in expected.get("absent", []):
                    self.assertNotIn(dep, main_deps)
                    self.assertNotIn(dep, test_deps)


if __name__ == "__main__":
    unittest.main()
