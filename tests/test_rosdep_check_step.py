"""Unit tests for ``RosdepCheckStep`` early-return branches.

The step has two early-exit paths that are not exercised by any other
test suite: the config-flag check at the top, and the empty-deps check
after dependency retrieval. These tests pin both behaviours so future
refactors can't silently swallow the gate."""

import os
import tempfile
import unittest
from unittest.mock import MagicMock

import lxml.etree as ET

from package_xml_validation.helpers.pkg_xml_formatter import PackageXmlFormatter
from package_xml_validation.helpers.validation_steps import ValidationConfig
from package_xml_validation.helpers.validation_steps.rosdep_check_step import (
    RosdepCheckStep,
)


def _config(**overrides):
    base = {
        "check_only": True,
        "auto_fill_missing_deps": False,
        "check_rosdeps": True,
        "compare_with_cmake": False,
        "strict_cmake_checking": False,
        "missing_deps_only": False,
        "ignore_formatting_errors": False,
    }
    base.update(overrides)
    return ValidationConfig(**base)


def _parse(xml: str):
    return ET.fromstring(xml.encode("utf-8"))


_PKG_WITH_DEP = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <exec_depend>rclcpp</exec_depend>
</package>
"""

_PKG_NO_DEPS = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
</package>
"""


class TestRosdepCheckStepEarlyReturn(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory(prefix="rosdep_check_step_")
        self.addCleanup(tmp.cleanup)
        self.fake_xml = os.path.join(tmp.name, "demo", "package.xml")

    def _make_step(self, config: ValidationConfig, rosdep: MagicMock):
        formatter = PackageXmlFormatter(check_only=config.check_only)
        return RosdepCheckStep(
            config=config, formatter=formatter, rosdep_validator=rosdep
        )

    def test_returns_early_when_check_rosdeps_disabled(self):
        rosdep = MagicMock()
        step = self._make_step(_config(check_rosdeps=False), rosdep)
        result = step.perform_check(_parse(_PKG_WITH_DEP), self.fake_xml)

        self.assertTrue(result.valid)
        self.assertEqual(result.critical_errors, [])
        rosdep.check_rosdeps_and_local_pkgs.assert_not_called()

    def test_returns_early_when_missing_deps_only(self):
        rosdep = MagicMock()
        step = self._make_step(
            _config(check_rosdeps=True, missing_deps_only=True), rosdep
        )
        result = step.perform_check(_parse(_PKG_WITH_DEP), self.fake_xml)

        self.assertTrue(result.valid)
        self.assertEqual(result.critical_errors, [])
        rosdep.check_rosdeps_and_local_pkgs.assert_not_called()

    def test_returns_early_when_no_deps_in_root(self):
        rosdep = MagicMock()
        step = self._make_step(_config(check_rosdeps=True), rosdep)
        result = step.perform_check(_parse(_PKG_NO_DEPS), self.fake_xml)

        self.assertTrue(result.valid)
        self.assertEqual(result.critical_errors, [])
        rosdep.check_rosdeps_and_local_pkgs.assert_not_called()


if __name__ == "__main__":
    unittest.main()
