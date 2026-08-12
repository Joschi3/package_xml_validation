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
        "python_composable_nodes.launch.py": ["image_proc", "lifecycle_demo"],
        "python_find_package_prefix.launch.py": [
            "foo_pkg",
            "bar_pkg",
            "baz_pkg",
            "qux_pkg",
        ],
        "xml_composable_container.launch.xml": [
            "comp_a",
            "cont_b",
            "find_prefix_pkg",
        ],
        "better_launch_example.launch.py": ["bl_pkg_a", "bl_pkg_b"],
        # `pkg:` and `package =` are YAML and TOML syntax; in Python they are an annotation
        # and an assignment.
        "python_annotation_and_literal.py": [],
        # A key merely *ending* in `pkg` is not a reference.
        "yaml_keys_ending_in_pkg.launch.yml": ["real_package", "quoted_package"],
        # ... but the anchor allows the `- ` of a sequence item, which is how every hector
        # component definition names its package.
        "yaml_sequence_item_package.yaml": [
            "sequence_item_package",
            "another_sequence_package",
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

    def test_scan_directory_with_toml(self):
        """better_launch .toml files under a launch/ dir contribute deps; .toml
        outside a launch/ dir must not."""
        pkg_dir = os.path.join(
            os.path.dirname(__file__),
            "examples",
            "launch_examples",
            "better_launch_toml",
        )
        found = set(scan_files(pkg_dir))
        self.assertIn("toml_pkg_a", found)
        self.assertNotIn("should_be_ignored", found)
        self.assertNotIn("must_not_match", found)

    def test_scan_file_toml_outside_launch_dir_is_skipped(self):
        """Direct ``scan_file`` call on a TOML file living outside a
        ``launch/`` directory must not contribute any packages, even when
        the contents would otherwise match the TOML regex."""
        import tempfile

        with tempfile.TemporaryDirectory(prefix="toml_outside_launch_") as tmp:
            config_dir = os.path.join(tmp, "config")
            os.makedirs(config_dir)
            toml_path = os.path.join(config_dir, "stuff.toml")
            with open(toml_path, "w", encoding="utf-8") as f:
                f.write('package = "should_not_be_found"\n')

            found: set[str] = set()
            scan_file(toml_path, found)
            self.assertEqual(found, set())


if __name__ == "__main__":
    unittest.main()


class TestPatternsAreScopedToTheirFormat(unittest.TestCase):
    """A pattern only applies to the languages it is syntax for.

    The Python fixture writes each form at the start of a line, where anchoring alone would let
    it through, so these fail if the format scoping is removed and not only if the anchor is.
    """

    BASE_DIR = os.path.join(os.path.dirname(__file__), "examples", "launch_examples")

    def scan(self, filename):
        found = set()
        scan_file(os.path.join(self.BASE_DIR, filename), found)
        return found

    def test_a_python_annotation_is_not_a_mapping_key(self):
        """`pkg: str = "hello"` at module level: a YAML key everywhere but in Python."""
        self.assertNotIn("str", self.scan("python_annotation_and_literal.py"))

    def test_a_python_assignment_is_not_the_toml_form(self):
        """`package = "x"` names a package in better_launch TOML and nothing in Python."""
        self.assertNotIn(
            "sneaky_toml_form", self.scan("python_annotation_and_literal.py")
        )

    def test_a_string_containing_pkg_is_not_a_package(self):
        """A `"pkg:robot.launch.yaml"` literal names a file, not a dependency."""
        self.assertNotIn("robot", self.scan("python_annotation_and_literal.py"))

    def test_a_key_merely_ending_in_pkg_is_not_a_reference(self):
        """`mypkg: foo` is a key of its own, not the `pkg:` key."""
        found = self.scan("yaml_keys_ending_in_pkg.launch.yml")
        self.assertNotIn("not_a_package", found)
        self.assertNotIn("also_not_a_package", found)

    def test_the_real_references_beside_them_still_match(self):
        """The point is precision, not silence."""
        found = self.scan("yaml_keys_ending_in_pkg.launch.yml")
        self.assertIn("real_package", found)
        self.assertIn("quoted_package", found)

    def test_a_sequence_item_key_still_matches(self):
        """`- package: x` is how every hector component definition names its package."""
        found = self.scan("yaml_sequence_item_package.yaml")
        self.assertEqual({"sequence_item_package", "another_sequence_package"}, found)

    def test_every_format_specific_pattern_is_withheld_from_the_others(self):
        """The mechanism itself, rather than one fixture's worth of it: whatever a pattern
        declares it is syntax for is exactly what `_patterns_for` hands it."""
        from package_xml_validation.helpers.find_launch_dependencies import (
            ANY_FORMAT,
            COMPILED_PATTERNS,
            PATTERNS,
            _patterns_for,
        )

        specific = [i for i, (_, fmts) in enumerate(PATTERNS) if fmts != ANY_FORMAT]
        self.assertTrue(specific, "no format-specific pattern left to check")

        for fmt in ANY_FORMAT:
            applied = {i for _, i in _patterns_for(fmt)}
            for i in specific:
                declared = fmt in COMPILED_PATTERNS[i][1]
                self.assertEqual(
                    declared, i in applied, f"pattern {PATTERNS[i][0]!r} in {fmt}"
                )
