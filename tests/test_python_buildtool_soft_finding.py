"""Tests for the softened ament_python <buildtool_depend> rule.

Per the ROS 2 rolling "Creating a package" tutorial, an ament_python
package.xml does not need to declare <buildtool_depend>ament_python</…>;
only <export><build_type>ament_python</build_type></export> is required.
We surface the missing tag as a warning rather than failing validation.
"""

import tempfile
import unittest
from pathlib import Path
from unittest import mock

import lxml.etree as ET

from package_xml_validation.helpers.validation_steps import (
    BuildToolDependStep,
    ValidationConfig,
)


def _config(**overrides):
    base = dict(
        check_only=True,
        auto_fill_missing_deps=False,
        check_rosdeps=False,
        compare_with_cmake=False,
        strict_cmake_checking=False,
        missing_deps_only=False,
        ignore_formatting_errors=False,
    )
    base.update(overrides)
    return ValidationConfig(**base)


def _python_pkg_dir(xml: str) -> Path:
    """Build a directory whose layout pushes ``get_package_type`` to PYTHON_PKG."""
    tmp = Path(tempfile.mkdtemp(prefix="python_buildtool_"))
    (tmp / "setup.py").write_text("from setuptools import setup\nsetup()\n")
    (tmp / "package.xml").write_text(xml, encoding="utf-8")
    return tmp


def _cmake_pkg_dir(xml: str) -> Path:
    tmp = Path(tempfile.mkdtemp(prefix="cmake_buildtool_"))
    (tmp / "CMakeLists.txt").write_text("project(demo)\n")
    (tmp / "package.xml").write_text(xml, encoding="utf-8")
    return tmp


class TestPythonBuildtoolSoftFinding(unittest.TestCase):
    PYTHON_NO_BUILDTOOL = """<?xml version="1.0"?>
<package format="3">
  <name>python_pkg</name>
  <version>0.0.1</version>
  <description>demo</description>
  <maintainer email="a@example.com">Alex</maintainer>
  <license>Apache-2.0</license>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"""

    PYTHON_WRONG_BUILDTOOL = """<?xml version="1.0"?>
<package format="3">
  <name>python_pkg</name>
  <version>0.0.1</version>
  <description>demo</description>
  <maintainer email="a@example.com">Alex</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"""

    def test_python_pkg_missing_buildtool_is_warning_not_error(self):
        pkg = _python_pkg_dir(self.PYTHON_NO_BUILDTOOL)
        root = ET.fromstring(self.PYTHON_NO_BUILDTOOL.encode("utf-8"))
        step = BuildToolDependStep(_config(), formatter=mock.Mock())
        result = step.perform_check(root, str(pkg / "package.xml"))
        self.assertTrue(result.valid)
        self.assertEqual(result.errors, [])
        self.assertEqual(result.critical_errors, [])
        self.assertEqual(len(result.warnings), 1)

    def test_python_pkg_wrong_buildtool_is_warning_not_error(self):
        pkg = _python_pkg_dir(self.PYTHON_WRONG_BUILDTOOL)
        root = ET.fromstring(self.PYTHON_WRONG_BUILDTOOL.encode("utf-8"))
        step = BuildToolDependStep(_config(), formatter=mock.Mock())
        result = step.perform_check(root, str(pkg / "package.xml"))
        self.assertTrue(result.valid)
        self.assertEqual(result.errors, [])
        self.assertEqual(len(result.warnings), 1)

    def test_python_pkg_auto_fill_still_inserts(self):
        pkg = _python_pkg_dir(self.PYTHON_NO_BUILDTOOL)
        root = ET.fromstring(self.PYTHON_NO_BUILDTOOL.encode("utf-8"))
        formatter = mock.Mock()
        step = BuildToolDependStep(
            _config(check_only=False, auto_fill_missing_deps=True), formatter=formatter
        )
        result = step.perform_check(root, str(pkg / "package.xml"))
        self.assertTrue(result.changed)
        formatter.add_buildtool_depends.assert_called_once_with(root, ["ament_python"])

    def test_python_pkg_no_auto_fill_still_warns(self):
        """Without --auto-fill-missing-deps and not in check-only mode,
        we used to emit a critical_error for Python packages. Soft
        finding now: warning, no critical_error."""
        pkg = _python_pkg_dir(self.PYTHON_NO_BUILDTOOL)
        root = ET.fromstring(self.PYTHON_NO_BUILDTOOL.encode("utf-8"))
        step = BuildToolDependStep(
            _config(check_only=False, auto_fill_missing_deps=False),
            formatter=mock.Mock(),
        )
        result = step.perform_check(root, str(pkg / "package.xml"))
        self.assertTrue(result.valid)
        self.assertEqual(result.critical_errors, [])
        self.assertEqual(len(result.warnings), 1)

    def test_cmake_pkg_missing_buildtool_still_errors(self):
        """Regression: CMake packages must still hard-fail without ament_cmake."""
        cmake_xml = """<?xml version="1.0"?>
<package format="3">
  <name>cmake_pkg</name>
  <version>0.0.1</version>
  <description>demo</description>
  <maintainer email="a@example.com">Alex</maintainer>
  <license>Apache-2.0</license>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
        pkg = _cmake_pkg_dir(cmake_xml)
        root = ET.fromstring(cmake_xml.encode("utf-8"))
        step = BuildToolDependStep(_config(), formatter=mock.Mock())
        result = step.perform_check(root, str(pkg / "package.xml"))
        self.assertFalse(result.valid)
        self.assertTrue(any("ament_cmake" in e for e in result.errors))


if __name__ == "__main__":
    unittest.main()
