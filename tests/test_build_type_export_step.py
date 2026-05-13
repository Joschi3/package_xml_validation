"""Targeted tests for BuildTypeExportStep on edge-case package types."""

import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

import lxml.etree as ET

from package_xml_validation.helpers.validation_steps import (
    BuildTypeExportStep,
    ValidationConfig,
)


def _config(**overrides):
    defaults = {
        "check_only": False,
        "auto_fill_missing_deps": True,
        "check_rosdeps": False,
        "compare_with_cmake": False,
        "strict_cmake_checking": False,
        "missing_deps_only": False,
        "ignore_formatting_errors": False,
    }
    defaults.update(overrides)
    return ValidationConfig(**defaults)


def _root_without_export() -> ET._Element:
    return ET.fromstring(b"<package format='3'><name>x</name></package>")


class TestBuildTypeExportStepUnknownType(unittest.TestCase):
    """A manifest-only package (no CMakeLists.txt, no setup.py) has
    PackageType.UNKNOWN. The step must not auto-fill or flag a build_type."""

    def setUp(self):
        self.tmpdir = Path(tempfile.mkdtemp(prefix="build_type_unknown_"))
        self.pkg_dir = self.tmpdir / "manifest_only"
        self.pkg_dir.mkdir()
        self.xml_file = str(self.pkg_dir / "package.xml")

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_unknown_type_skips_without_mutation(self):
        formatter = MagicMock()
        step = BuildTypeExportStep(_config(), formatter)
        root = _root_without_export()

        result = step.perform_check(root, self.xml_file)

        self.assertTrue(result.valid)
        self.assertFalse(result.changed)
        self.assertEqual(result.errors, [])
        self.assertEqual(result.critical_errors, [])
        formatter.add_build_type_export.assert_not_called()

    def test_unknown_type_in_check_only_does_not_flag(self):
        formatter = MagicMock()
        step = BuildTypeExportStep(_config(check_only=True), formatter)
        root = _root_without_export()

        result = step.perform_check(root, self.xml_file)

        self.assertTrue(result.valid)
        self.assertEqual(result.errors, [])

    def test_unknown_type_without_auto_fill_does_not_critical(self):
        formatter = MagicMock()
        step = BuildTypeExportStep(_config(auto_fill_missing_deps=False), formatter)
        root = _root_without_export()

        result = step.perform_check(root, self.xml_file)

        self.assertTrue(result.valid)
        self.assertEqual(result.critical_errors, [])
        formatter.add_build_type_export.assert_not_called()


class TestBuildTypeExportStepCMakePackage(unittest.TestCase):
    """Tests for a CMake pkg (``CMakeLists.txt`` present). The expected
    ``<export><build_type>`` is ``ament_cmake``; the step must report or
    auto-fill missing/incorrect entries depending on the config flags.
    """

    def setUp(self):
        self.tmpdir = Path(tempfile.mkdtemp(prefix="build_type_cmake_"))
        self.pkg_dir = self.tmpdir / "cmake_pkg"
        self.pkg_dir.mkdir()
        (self.pkg_dir / "CMakeLists.txt").write_text(
            "cmake_minimum_required(VERSION 3.5)\n"
        )
        self.xml_file = str(self.pkg_dir / "package.xml")

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def _root_with_wrong_build_type(self) -> ET._Element:
        return ET.fromstring(
            b"<package format='3'>"
            b"<name>x</name>"
            b"<export><build_type>ament_python</build_type></export>"
            b"</package>"
        )

    def test_check_only_missing_export_reports_error(self):
        formatter = MagicMock()
        step = BuildTypeExportStep(_config(check_only=True), formatter)
        root = _root_without_export()

        result = step.perform_check(root, self.xml_file)

        self.assertFalse(result.valid)
        self.assertEqual(len(result.errors), 1)
        self.assertIn("Missing <export> tag", result.errors[0])
        self.assertEqual(result.critical_errors, [])
        formatter.add_build_type_export.assert_not_called()

    def test_check_only_wrong_build_type_reports_error(self):
        formatter = MagicMock()
        step = BuildTypeExportStep(_config(check_only=True), formatter)
        root = self._root_with_wrong_build_type()

        result = step.perform_check(root, self.xml_file)

        self.assertFalse(result.valid)
        self.assertEqual(len(result.errors), 1)
        self.assertIn("Incorrect <build_type>", result.errors[0])
        formatter.add_build_type_export.assert_not_called()

    def test_no_auto_fill_yields_critical_error(self):
        """``check_only=False`` and ``auto_fill_missing_deps=False`` is
        the "can't fix it for you, but it must be fixed" path."""
        formatter = MagicMock()
        step = BuildTypeExportStep(
            _config(check_only=False, auto_fill_missing_deps=False), formatter
        )
        root = _root_without_export()

        result = step.perform_check(root, self.xml_file)

        self.assertFalse(result.valid)
        self.assertEqual(result.errors, [])
        self.assertEqual(len(result.critical_errors), 1)
        self.assertIn("Cannot auto-fill", result.critical_errors[0])
        formatter.add_build_type_export.assert_not_called()

    def test_auto_fill_inserts_and_marks_changed(self):
        formatter = MagicMock()
        step = BuildTypeExportStep(
            _config(check_only=False, auto_fill_missing_deps=True), formatter
        )
        root = _root_without_export()

        result = step.perform_check(root, self.xml_file)

        self.assertFalse(result.valid)
        self.assertTrue(result.changed)
        self.assertEqual(len(result.warnings), 1)
        self.assertIn("Auto-filling", result.warnings[0])
        formatter.add_build_type_export.assert_called_once_with(root, "ament_cmake")


if __name__ == "__main__":
    unittest.main()
