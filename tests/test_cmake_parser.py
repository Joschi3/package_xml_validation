import os
import unittest

from package_xml_validation.helpers.cmake_parsers import (
    read_deps_from_cmake_file,
    resolve_for_each,
    retrieve_cmake_dependencies,
)

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
                    "fastcdr",
                    "fmt",
                    "spdlog",
                ],
                "test": ["ament_cmake_gtest", "benchmark"],
                "absent": ["OpenMP", "Threads", "ament_cmake", "Python3"],
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


class TestFindPackageQuietRule(unittest.TestCase):
    """``find_package(... QUIET)`` without ``REQUIRED`` is optional — the
    build does not fail if the package is missing, so the parser must
    not report it as a dep. Adding ``REQUIRED`` flips the behaviour."""

    def test_quiet_without_required_is_skipped(self):
        lines = ["find_package(OptionalPkg QUIET)"]
        main_deps, test_deps = retrieve_cmake_dependencies(lines)
        self.assertEqual(main_deps, [])
        self.assertEqual(test_deps, [])

    def test_quiet_with_required_is_kept(self):
        lines = ["find_package(MustHavePkg QUIET REQUIRED)"]
        main_deps, _ = retrieve_cmake_dependencies(lines)
        self.assertEqual(main_deps, ["MustHavePkg"])

    def test_required_without_quiet_is_kept(self):
        lines = ["find_package(NeededPkg REQUIRED)"]
        main_deps, _ = retrieve_cmake_dependencies(lines)
        self.assertEqual(main_deps, ["NeededPkg"])


class TestResolveForEach(unittest.TestCase):
    """``resolve_for_each`` expands ``foreach(... IN LISTS/ITEMS ...)``
    blocks so dependencies declared inside loops are still discovered.
    These are pure-input/pure-output transformations on CMake lines."""

    def test_foreach_in_lists_expands_set_variable(self):
        input_lines = [
            "set(LIBS a b c)",
            "foreach(L IN LISTS LIBS)",
            "find_package(${L} REQUIRED)",
            "endforeach()",
        ]
        expanded = resolve_for_each(input_lines)
        # Loop-body lines are replicated once per value, with the loop
        # variable substituted in place of ``${L}``.
        self.assertIn("find_package(a REQUIRED)", expanded)
        self.assertIn("find_package(b REQUIRED)", expanded)
        self.assertIn("find_package(c REQUIRED)", expanded)

    def test_foreach_expansion_round_trips_through_retrieve(self):
        """End-to-end: foreach-driven find_package calls become deps."""
        lines = [
            "set(LIBS Alpha Beta)",
            "foreach(L IN LISTS LIBS)",
            "find_package(${L} REQUIRED)",
            "endforeach()",
        ]
        # The retrieve step does not call resolve_for_each on its own;
        # callers chain them via read_cmake_file. So we run it explicitly.
        expanded = resolve_for_each(lines)
        main_deps, _ = retrieve_cmake_dependencies(expanded)
        self.assertCountEqual(main_deps, ["Alpha", "Beta"])

    def test_foreach_in_items_uses_inline_values(self):
        """``IN ITEMS`` accepts a bare identifier; the regex requires it
        to look like a variable name, so this exercises the
        ``variables.get(..., [])`` fallback when the name isn't a
        previously ``set()`` variable."""
        input_lines = [
            "foreach(P IN ITEMS unknown_var)",
            "find_package(${P} REQUIRED)",
            "endforeach()",
        ]
        expanded = resolve_for_each(input_lines)
        # With no matching set(), the loop body is emitted unchanged
        # (the empty value list produces zero replications, leaving only
        # any non-template lines). This pins the don't-crash behaviour.
        self.assertNotIn("find_package(unknown_var REQUIRED)", expanded)

    def test_lines_outside_foreach_are_passed_through_verbatim(self):
        input_lines = [
            "find_package(rclcpp REQUIRED)",
            "set(IGNORED x)",
            "find_package(std_msgs REQUIRED)",
        ]
        expanded = resolve_for_each(input_lines)
        self.assertIn("find_package(rclcpp REQUIRED)", expanded)
        self.assertIn("find_package(std_msgs REQUIRED)", expanded)


if __name__ == "__main__":
    unittest.main()
