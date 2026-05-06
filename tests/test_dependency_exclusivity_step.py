"""Tests for REP-149 <depend> exclusivity enforcement."""

import os
import tempfile
import unittest

import lxml.etree as ET

from package_xml_validation.helpers.validation_steps import (
    DependencyExclusivityStep,
    ValidationConfig,
)


def _config(**overrides):
    base = {
        "check_only": True,
        "auto_fill_missing_deps": False,
        "check_rosdeps": False,
        "compare_with_cmake": False,
        "strict_cmake_checking": False,
        "missing_deps_only": False,
        "ignore_formatting_errors": False,
    }
    base.update(overrides)
    return ValidationConfig(**base)


def _parse(xml: str):
    return ET.fromstring(xml.encode("utf-8"))


class TestDependencyExclusivityStep(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory(prefix="dep_exclusivity_")
        self.addCleanup(tmp.cleanup)
        self.fake_xml = os.path.join(tmp.name, "demo", "package.xml")

    def test_no_overlap_passes(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <depend>rclcpp</depend>
  <build_depend>cmake</build_depend>
  <test_depend>gtest</test_depend>
</package>
"""
        step = DependencyExclusivityStep(_config())
        result = step.perform_check(_parse(xml), self.fake_xml)
        self.assertTrue(result.valid)
        self.assertEqual(result.errors, [])

    def test_overlap_with_build_depend_reports_error(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <depend>foo</depend>
  <build_depend>foo</build_depend>
</package>
"""
        step = DependencyExclusivityStep(_config())
        result = step.perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)
        self.assertTrue(any("build_depend" in e and "foo" in e for e in result.errors))
        self.assertFalse(result.changed)

    def test_overlap_with_build_export_depend_reports_error(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <depend>foo</depend>
  <build_export_depend>foo</build_export_depend>
</package>
"""
        step = DependencyExclusivityStep(_config())
        result = step.perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)
        self.assertTrue(
            any("build_export_depend" in e and "foo" in e for e in result.errors)
        )

    def test_overlap_with_exec_depend_reports_error(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <depend>foo</depend>
  <exec_depend>foo</exec_depend>
</package>
"""
        step = DependencyExclusivityStep(_config())
        result = step.perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)
        self.assertTrue(any("exec_depend" in e and "foo" in e for e in result.errors))

    def test_test_depend_overlap_is_allowed(self):
        """REP-149's exclusivity rule does NOT cover <test_depend>."""
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <depend>foo</depend>
  <test_depend>foo</test_depend>
</package>
"""
        step = DependencyExclusivityStep(_config())
        result = step.perform_check(_parse(xml), self.fake_xml)
        self.assertTrue(result.valid)
        self.assertEqual(result.errors, [])

    def test_buildtool_depend_overlap_is_allowed(self):
        """REP-149's exclusivity rule does NOT cover <buildtool_depend>."""
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <depend>foo</depend>
  <buildtool_depend>foo</buildtool_depend>
</package>
"""
        step = DependencyExclusivityStep(_config())
        result = step.perform_check(_parse(xml), self.fake_xml)
        self.assertTrue(result.valid)

    def test_auto_fix_collapses_unconditional_overlap(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <depend>foo</depend>
  <build_depend>foo</build_depend>
  <exec_depend>foo</exec_depend>
</package>
"""
        root = _parse(xml)
        step = DependencyExclusivityStep(
            _config(check_only=False, auto_fill_missing_deps=True)
        )
        result = step.perform_check(root, self.fake_xml)
        self.assertTrue(result.changed)
        self.assertFalse(result.valid)
        # Granular tags removed; <depend>foo</depend> remains.
        self.assertEqual(len(root.findall("depend")), 1)
        self.assertEqual(len(root.findall("build_depend")), 0)
        self.assertEqual(len(root.findall("exec_depend")), 0)

    def test_auto_fix_skips_when_granular_has_extra_condition(self):
        """If <build_depend> is conditional but <depend> is not, the
        granular tag is *narrower*. Removing it would change semantics."""
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <depend>foo</depend>
  <build_depend condition="$ROS_DISTRO == 'jazzy'">foo</build_depend>
</package>
"""
        root = _parse(xml)
        step = DependencyExclusivityStep(
            _config(check_only=False, auto_fill_missing_deps=True)
        )
        result = step.perform_check(root, self.fake_xml)
        self.assertFalse(result.valid)
        self.assertFalse(result.changed)
        self.assertEqual(len(root.findall("build_depend")), 1)
        self.assertTrue(any("manually" in e for e in result.errors))

    def test_auto_fix_collapses_when_conditions_match(self):
        """Same condition on both → genuinely redundant, safe to remove."""
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <depend condition="$ROS_DISTRO == 'jazzy'">foo</depend>
  <build_depend condition="$ROS_DISTRO == 'jazzy'">foo</build_depend>
</package>
"""
        root = _parse(xml)
        step = DependencyExclusivityStep(
            _config(check_only=False, auto_fill_missing_deps=True)
        )
        result = step.perform_check(root, self.fake_xml)
        self.assertTrue(result.changed)
        self.assertEqual(len(root.findall("build_depend")), 0)

    def test_skipped_under_missing_deps_only(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <depend>foo</depend>
  <build_depend>foo</build_depend>
</package>
"""
        step = DependencyExclusivityStep(_config(missing_deps_only=True))
        result = step.perform_check(_parse(xml), self.fake_xml)
        self.assertTrue(result.valid)
        self.assertEqual(result.errors, [])


if __name__ == "__main__":
    unittest.main()
