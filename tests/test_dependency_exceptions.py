import os
import unittest
import tempfile
import shutil
from unittest.mock import MagicMock

import lxml.etree as ET

from package_xml_validation.helpers.exception_parser import (
    DependencyExceptions,
    parse_exceptions,
)
from package_xml_validation.package_xml_validator import PackageXmlValidator


class TestParseExceptions(unittest.TestCase):
    """Unit tests for the exception_parser module."""

    def _make_root(self, xml_str: str):
        return ET.fromstring(xml_str)

    def test_no_comments(self):
        root = self._make_root('<package format="3"><name>test</name></package>')
        exceptions = parse_exceptions(root)
        self.assertEqual(exceptions.ignored_deps, frozenset())

    def test_single_ignore_comment(self):
        root = self._make_root(
            '<package format="3">'
            "<!-- validator:ignore rclpy -->"
            "<name>test</name>"
            "</package>"
        )
        exceptions = parse_exceptions(root)
        self.assertEqual(exceptions.ignored_deps, frozenset({"rclpy"}))

    def test_multiple_deps_in_one_comment(self):
        root = self._make_root(
            '<package format="3">'
            "<!-- validator:ignore rclpy ament_lint_auto sensor_msgs -->"
            "<name>test</name>"
            "</package>"
        )
        exceptions = parse_exceptions(root)
        self.assertEqual(
            exceptions.ignored_deps,
            frozenset({"rclpy", "ament_lint_auto", "sensor_msgs"}),
        )

    def test_multiple_ignore_comments_merged(self):
        root = self._make_root(
            '<package format="3">'
            "<!-- validator:ignore rclpy -->"
            "<!-- validator:ignore sensor_msgs -->"
            "<name>test</name>"
            "</package>"
        )
        exceptions = parse_exceptions(root)
        self.assertEqual(exceptions.ignored_deps, frozenset({"rclpy", "sensor_msgs"}))

    def test_non_matching_comments_ignored(self):
        root = self._make_root(
            '<package format="3">'
            "<!-- This is a regular comment -->"
            "<!-- validator:ignore rclpy -->"
            "<!-- Another comment -->"
            "<name>test</name>"
            "</package>"
        )
        exceptions = parse_exceptions(root)
        self.assertEqual(exceptions.ignored_deps, frozenset({"rclpy"}))

    def test_empty_ignore_directive(self):
        """validator:ignore with no deps should produce empty set."""
        root = self._make_root(
            '<package format="3"><!-- validator:ignore --><name>test</name></package>'
        )
        exceptions = parse_exceptions(root)
        self.assertEqual(exceptions.ignored_deps, frozenset())


class TestDependencyExceptions(unittest.TestCase):
    """Unit tests for the DependencyExceptions dataclass."""

    def test_is_ignored_match(self):
        exc = DependencyExceptions(ignored_deps=frozenset({"rclpy", "sensor_msgs"}))
        self.assertTrue(exc.is_ignored("rclpy"))
        self.assertTrue(exc.is_ignored("sensor_msgs"))

    def test_is_ignored_no_match(self):
        exc = DependencyExceptions(ignored_deps=frozenset({"rclpy"}))
        self.assertFalse(exc.is_ignored("sensor_msgs"))

    def test_empty_exceptions(self):
        exc = DependencyExceptions()
        self.assertFalse(exc.is_ignored("anything"))


class TestCMakeComparisonWithExceptions(unittest.TestCase):
    """Integration test: CMake comparison step respects validator:ignore comments."""

    @classmethod
    def setUpClass(cls):
        current_dir = os.path.dirname(__file__)
        cls.examples_dir = os.path.join(
            current_dir, "examples", "exception_tests", "cmake_exceptions"
        )

    def setUp(self):
        self.test_dir = tempfile.mkdtemp(prefix="xml_exception_tests_")
        shutil.copytree(self.examples_dir, self.test_dir, dirs_exist_ok=True)

    def tearDown(self):
        shutil.rmtree(self.test_dir)

    def _make_validator(self, **kwargs):
        validator = PackageXmlValidator(
            check_only=kwargs.get("check_only", True),
            verbose=True,
            auto_fill_missing_deps=kwargs.get("auto_fill_missing_deps", False),
            check_rosdeps=True,
            compare_with_cmake=True,
        )
        validator.rosdep_validator = MagicMock()
        validator.rosdep_validator.check_rosdeps_and_local_pkgs.return_value = []
        validator.rosdep_validator.resolve_cmake_dependency.side_effect = lambda x: x
        return validator

    def test_ignored_dep_not_flagged(self):
        """A package.xml with validator:ignore for the missing dep should pass."""
        xml_file = os.path.join(self.test_dir, "pkg_with_exception", "package.xml")
        validator = self._make_validator(check_only=True)
        valid = validator.check_and_format_files([xml_file])
        self.assertTrue(valid, "Package with validator:ignore should pass validation.")

    def test_without_exception_fails(self):
        """Same package without the ignore comment should fail."""
        xml_file = os.path.join(self.test_dir, "pkg_without_exception", "package.xml")
        validator = self._make_validator(check_only=True)
        valid = validator.check_and_format_files([xml_file])
        self.assertFalse(
            valid, "Package without validator:ignore should fail validation."
        )

    def test_global_ignore_deps_cli(self):
        """The --ignore-deps CLI argument should suppress the missing dep error."""
        xml_file = os.path.join(self.test_dir, "pkg_without_exception", "package.xml")
        validator = self._make_validator(check_only=True)
        # Simulate --ignore-deps urdf
        validator.global_ignored_deps = frozenset({"urdf"})
        valid = validator.check_and_format_files([xml_file])
        self.assertTrue(valid, "Global --ignore-deps should suppress the missing dep.")

    def test_auto_fill_skips_ignored_deps(self):
        """Auto-fill should not add ignored dependencies."""
        xml_file = os.path.join(self.test_dir, "pkg_with_exception", "package.xml")

        validator = self._make_validator(check_only=False, auto_fill_missing_deps=True)
        validator.check_and_format_files([xml_file])

        with open(xml_file) as f:
            updated_content = f.read()

        # The ignored dep (urdf) should NOT have been added
        self.assertNotIn("<depend>urdf</depend>", updated_content)


class TestLaunchDepsWithExceptions(unittest.TestCase):
    """Integration test: Launch dependency step respects validator:ignore comments."""

    @classmethod
    def setUpClass(cls):
        current_dir = os.path.dirname(__file__)
        cls.examples_dir = os.path.join(
            current_dir, "examples", "exception_tests", "launch_exceptions"
        )

    def setUp(self):
        self.test_dir = tempfile.mkdtemp(prefix="xml_launch_exception_tests_")
        shutil.copytree(self.examples_dir, self.test_dir, dirs_exist_ok=True)

    def tearDown(self):
        shutil.rmtree(self.test_dir)

    def _make_validator(self, **kwargs):
        validator = PackageXmlValidator(
            check_only=kwargs.get("check_only", True),
            verbose=True,
            auto_fill_missing_deps=kwargs.get("auto_fill_missing_deps", False),
            check_rosdeps=True,
            compare_with_cmake=False,
        )
        validator.rosdep_validator = MagicMock()
        validator.rosdep_validator.check_rosdeps_and_local_pkgs.return_value = []
        return validator

    def test_ignored_launch_dep_not_flagged(self):
        """A package.xml with validator:ignore for a launch dep should pass."""
        xml_file = os.path.join(self.test_dir, "pkg_with_exception", "package.xml")
        validator = self._make_validator(check_only=True)
        valid = validator.check_and_format_files([xml_file])
        self.assertTrue(
            valid, "Package with validator:ignore for launch dep should pass."
        )

    def test_without_exception_fails(self):
        """Same package without the ignore comment should fail."""
        xml_file = os.path.join(self.test_dir, "pkg_without_exception", "package.xml")
        validator = self._make_validator(check_only=True)
        valid = validator.check_and_format_files([xml_file])
        self.assertFalse(valid, "Package without validator:ignore should fail.")


if __name__ == "__main__":
    unittest.main()
