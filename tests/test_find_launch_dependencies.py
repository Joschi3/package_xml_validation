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
        # A pattern is a piece of syntax, and `pkg:` means different things in different
        # languages. In Python it is a type annotation or part of a string, never a mapping
        # key - reading it as one reported packages named `str` and `robot`.
        "python_annotation_and_literal.py": [],
        # Any key *ending* in `pkg` used to match, because nothing anchored the pattern.
        "yaml_keys_ending_in_pkg.launch.yml": ["real_package", "quoted_package"],
        # ... and the anchor has to allow the `- ` of a sequence item, or every hector
        # component definition would stop being scanned.
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

    The three cases below were all live false positives. They are asserted directly, rather
    than only through the example files, because the mechanism is the point: a Python file
    must not be searched for YAML mapping keys, whatever it happens to contain.
    """

    BASE_DIR = os.path.join(os.path.dirname(__file__), "examples", "launch_examples")

    def scan(self, filename):
        found = set()
        scan_file(os.path.join(self.BASE_DIR, filename), found)
        return found

    def test_a_python_type_annotation_is_not_a_package(self):
        """`def resolve(pkg: str, ...)` reported a package named `str`."""
        self.assertNotIn("str", self.scan("python_annotation_and_literal.py"))

    def test_a_string_containing_pkg_is_not_a_package(self):
        """`"pkg:robot.launch.yaml"` in a unit test reported a package named `robot`, and the
        hook then demanded a <test_depend> on it."""
        self.assertNotIn("robot", self.scan("python_annotation_and_literal.py"))

    def test_a_key_merely_ending_in_pkg_is_not_a_reference(self):
        """`mypkg: foo` matched, because nothing anchored the pattern to the start of a key."""
        found = self.scan("yaml_keys_ending_in_pkg.launch.yml")
        self.assertNotIn("not_a_package", found)
        self.assertNotIn("also_not_a_package", found)

    def test_the_real_references_beside_them_still_match(self):
        """The point of the fix is precision, not silence."""
        found = self.scan("yaml_keys_ending_in_pkg.launch.yml")
        self.assertIn("real_package", found)
        self.assertIn("quoted_package", found)

    def test_a_sequence_item_key_still_matches(self):
        """`- package: x` is how every hector component definition names its package, so the
        anchor has to allow the leading dash."""
        found = self.scan("yaml_sequence_item_package.yaml")
        self.assertEqual({"sequence_item_package", "another_sequence_package"}, found)

    def test_the_toml_form_stays_out_of_yaml_and_python(self):
        """`package = "x"` is a TOML form. It is also an ordinary assignment in Python, which
        is why it was scoped in the first place - that scoping is now the general rule.

        Selected by the pattern's own declared formats rather than by asking which patterns
        apply to TOML: the latter also returns the format-agnostic ones like
        `$(find-pkg-share x)`, which are meant to apply everywhere.
        """
        from package_xml_validation.helpers.find_launch_dependencies import (
            FORMAT_TOML,
            PATTERNS,
        )

        toml_only = [rx for rx, formats in PATTERNS if formats == {FORMAT_TOML}]

        self.assertTrue(toml_only, "no TOML-specific pattern left to check")
        for rx in toml_only:
            self.assertIn("package", rx)
            found = set()
            scan_file(
                os.path.join(self.BASE_DIR, "python_annotation_and_literal.py"), found
            )
            self.assertEqual(set(), found)
