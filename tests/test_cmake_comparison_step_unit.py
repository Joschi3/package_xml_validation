"""Unit tests for ``CMakeComparisonStep`` branches not covered by the
end-to-end ``PackageXmlValidator`` tests in ``test_cmake_comparison.py``."""

import os
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import lxml.etree as ET

from package_xml_validation.helpers.exception_parser import DependencyExceptions
from package_xml_validation.helpers.package_types import PackageType
from package_xml_validation.helpers.pkg_xml_formatter import PackageXmlFormatter
from package_xml_validation.helpers.validation_steps import ValidationConfig
from package_xml_validation.helpers.validation_steps.cmake_comparison_step import (
    CMakeComparisonStep,
)


def _config(**overrides):
    base = {
        "check_only": False,
        "auto_fill_missing_deps": False,
        "check_rosdeps": True,
        "compare_with_cmake": True,
        "strict_cmake_checking": False,
        "missing_deps_only": False,
        "ignore_formatting_errors": False,
    }
    base.update(overrides)
    return ValidationConfig(**base)


def _parse(xml: str):
    return ET.fromstring(xml.encode("utf-8"))


_CMAKE_PKG_XML = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <version>0.0.1</version>
  <description>demo</description>
  <maintainer email="a@example.com">a</maintainer>
  <license>BSD</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
</package>
"""


class _CMakePkgFixture:
    """Helper that lays out a fake CMake pkg dir (package.xml + optional
    CMakeLists.txt). ``cleanup`` removes the temp tree."""

    def __init__(self, with_cmakelists: bool, cmake_body: str = "") -> None:
        self.root = tempfile.mkdtemp(prefix="cmake_cmp_step_")
        self.pkg_dir = os.path.join(self.root, "demo")
        os.makedirs(self.pkg_dir)
        self.xml_path = os.path.join(self.pkg_dir, "package.xml")
        with open(self.xml_path, "w", encoding="utf-8") as f:
            f.write(_CMAKE_PKG_XML)
        if with_cmakelists:
            with open(
                os.path.join(self.pkg_dir, "CMakeLists.txt"), "w", encoding="utf-8"
            ) as f:
                f.write(cmake_body)

    def cleanup(self) -> None:
        import shutil

        shutil.rmtree(self.root, ignore_errors=True)


def _make_step(config: ValidationConfig, rosdep_validator: MagicMock):
    formatter = PackageXmlFormatter(check_only=config.check_only)
    return CMakeComparisonStep(
        config=config,
        formatter=formatter,
        rosdep_validator=rosdep_validator,
        exceptions=DependencyExceptions(),
    )


class TestMissingCMakeLists(unittest.TestCase):
    """Item 1: the branch where a CMake pkg has no ``CMakeLists.txt``.

    In practice ``get_package_type`` derives the type *from* the presence
    of ``CMakeLists.txt``, so this branch is reachable only when a caller
    forces ``PackageType.CMAKE_PKG`` for a directory that happens to be
    missing the file. We patch ``get_package_type`` accordingly to
    exercise the error/auto-fill messaging the step defines.
    """

    def _force_cmake_pkg_type(self):
        return patch(
            "package_xml_validation.helpers.validation_steps."
            "cmake_comparison_step.get_package_type",
            return_value=(PackageType.CMAKE_PKG, False),
        )

    def test_missing_cmakelists_reports_critical_error_when_not_auto_filling(self):
        fixture = _CMakePkgFixture(with_cmakelists=False)
        self.addCleanup(fixture.cleanup)

        step = _make_step(
            _config(auto_fill_missing_deps=False, check_only=True),
            MagicMock(),
        )
        with self._force_cmake_pkg_type():
            result = step.perform_check(_parse(_CMAKE_PKG_XML), fixture.xml_path)

        self.assertFalse(result.valid)
        self.assertEqual(result.errors, [])
        self.assertEqual(len(result.critical_errors), 1)
        self.assertIn("does not exist", result.critical_errors[0])

    def test_missing_cmakelists_reports_error_when_auto_filling(self):
        fixture = _CMakePkgFixture(with_cmakelists=False)
        self.addCleanup(fixture.cleanup)

        step = _make_step(
            _config(auto_fill_missing_deps=True, check_only=False),
            MagicMock(),
        )
        with self._force_cmake_pkg_type():
            result = step.perform_check(_parse(_CMAKE_PKG_XML), fixture.xml_path)

        self.assertFalse(result.valid)
        self.assertEqual(result.critical_errors, [])
        self.assertEqual(len(result.errors), 1)
        self.assertIn("does not exist", result.errors[0])


class TestUnresolvedDepReporting(unittest.TestCase):
    """Item 2: strict-vs-non-strict reporting for deps that can't be
    resolved and (optionally) have rosdep search candidates."""

    def _fixture_with_dep(self) -> _CMakePkgFixture:
        # CMake file exposes a single dep "Frobnicator" that the rosdep
        # validator will refuse to resolve.
        body = "find_package(Frobnicator REQUIRED)\n"
        return _CMakePkgFixture(with_cmakelists=True, cmake_body=body)

    def test_unresolved_dep_with_candidates_strict_is_critical_error(self):
        fixture = self._fixture_with_dep()
        self.addCleanup(fixture.cleanup)

        rosdep = MagicMock()
        rosdep.resolve_cmake_dependency.return_value = None
        rosdep.search_rosdep_candidates.return_value = ["foo_alt", "bar_alt"]

        step = _make_step(_config(strict_cmake_checking=True), rosdep)
        result = step.perform_check(_parse(_CMAKE_PKG_XML), fixture.xml_path)

        self.assertFalse(result.valid)
        self.assertEqual(result.warnings, [])
        self.assertEqual(len(result.critical_errors), 1)
        msg = result.critical_errors[0]
        self.assertIn("Frobnicator", msg)
        self.assertIn("rosdep search candidates: foo_alt, bar_alt", msg)

    def test_unresolved_dep_without_candidates_non_strict_is_warning(self):
        fixture = self._fixture_with_dep()
        self.addCleanup(fixture.cleanup)

        rosdep = MagicMock()
        rosdep.resolve_cmake_dependency.return_value = None
        rosdep.search_rosdep_candidates.return_value = []

        step = _make_step(_config(strict_cmake_checking=False), rosdep)
        result = step.perform_check(_parse(_CMAKE_PKG_XML), fixture.xml_path)

        # Warnings-only path leaves result.valid untouched (defaults True).
        self.assertTrue(result.valid)
        self.assertEqual(result.critical_errors, [])
        self.assertEqual(len(result.warnings), 1)
        msg = result.warnings[0]
        self.assertIn("Frobnicator", msg)
        self.assertIn("via rosdep resolve, mapping, or rosdep search", msg)


class TestEvaluateConditionsFallback(unittest.TestCase):
    """Item 3: with ``evaluate_conditions=False`` the step must read deps
    via the formatter rather than the condition-aware queries."""

    def test_evaluate_conditions_false_uses_formatter_retrieve(self):
        body = "find_package(rclcpp REQUIRED)\n"
        fixture = _CMakePkgFixture(with_cmakelists=True, cmake_body=body)
        self.addCleanup(fixture.cleanup)

        rosdep = MagicMock()
        # Pretend every CMake dep resolves to itself so the step doesn't
        # bail out before it inspects the xml-side deps.
        rosdep.resolve_cmake_dependency.side_effect = lambda x: x
        rosdep.search_rosdep_candidates.return_value = []

        config = _config(evaluate_conditions=False)
        formatter = PackageXmlFormatter(check_only=config.check_only)
        formatter.retrieve_build_dependencies = MagicMock(return_value=["rclcpp"])
        formatter.retrieve_test_dependencies = MagicMock(return_value=[])

        step = CMakeComparisonStep(
            config=config,
            formatter=formatter,
            rosdep_validator=rosdep,
            exceptions=DependencyExceptions(),
        )
        root = _parse(_CMAKE_PKG_XML)
        result = step.perform_check(root, fixture.xml_path)

        formatter.retrieve_build_dependencies.assert_called_once_with(root)
        formatter.retrieve_test_dependencies.assert_called_once_with(root)
        # rclcpp is in the xml-side list, so no missing-dep complaints.
        self.assertEqual(result.critical_errors, [])
        self.assertEqual(result.errors, [])


if __name__ == "__main__":
    unittest.main()
