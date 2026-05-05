"""End-to-end tests that condition-aware steps filter inactive entries."""

import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import lxml.etree as ET

from package_xml_validation.helpers.validation_steps import (
    BuildTypeExportStep,
    RosdepCheckStep,
    ValidationConfig,
)


def _config(**overrides):
    base = dict(
        check_only=True,
        auto_fill_missing_deps=False,
        check_rosdeps=True,
        compare_with_cmake=False,
        strict_cmake_checking=False,
        missing_deps_only=False,
        ignore_formatting_errors=False,
        evaluate_conditions=True,
    )
    base.update(overrides)
    return ValidationConfig(**base)


class _FakeRosdepValidator:
    def __init__(self, unresolvable):
        self._unresolvable = set(unresolvable)

    def check_rosdeps_and_local_pkgs(self, deps):
        return [d for d in deps if d in self._unresolvable]


class _FakeFormatter:
    def retrieve_all_dependencies(self, root):
        return [
            e.text.strip()
            for e in root
            if isinstance(e.tag, str) and "depend" in e.tag and e.text
        ]


class TestRosdepConditionFiltering(unittest.TestCase):
    XML = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <depend>real_pkg</depend>
  <exec_depend condition="$ROS_DISTRO == 'humble'">humble_only_pkg</exec_depend>
</package>
"""

    def _run(self, config, env):
        root = ET.fromstring(self.XML.encode("utf-8"))
        validator = _FakeRosdepValidator(unresolvable=["humble_only_pkg"])
        step = RosdepCheckStep(config, _FakeFormatter(), validator)
        with mock.patch.dict(os.environ, env, clear=True):
            return step.perform_check(root, "/tmp/demo/package.xml")

    def test_inactive_condition_filtered_out(self):
        result = self._run(_config(), {"ROS_DISTRO": "jazzy"})
        self.assertTrue(result.valid)
        self.assertEqual(result.critical_errors, [])

    def test_active_condition_still_validated(self):
        result = self._run(_config(), {"ROS_DISTRO": "humble"})
        self.assertFalse(result.valid)
        self.assertTrue(any("humble_only_pkg" in e for e in result.critical_errors))

    def test_ignore_conditions_evaluates_all(self):
        result = self._run(_config(evaluate_conditions=False), {"ROS_DISTRO": "jazzy"})
        self.assertFalse(result.valid)
        self.assertTrue(any("humble_only_pkg" in e for e in result.critical_errors))


class TestBuildTypeExportConditionFiltering(unittest.TestCase):
    """If a <build_type> has an inactive condition the step should treat
    it as absent and (in check-only mode) flag it as missing."""

    def _build_pkg_dir(self, xml: str):
        # CMakeLists.txt presence pushes get_package_type to CMAKE_PKG.
        tmp = Path(tempfile.mkdtemp(prefix="condition_aware_"))
        (tmp / "CMakeLists.txt").write_text("project(demo)\n", encoding="utf-8")
        xml_path = tmp / "package.xml"
        xml_path.write_text(xml, encoding="utf-8")
        return xml_path

    def _run(self, xml: str, env: dict, evaluate_conditions: bool = True):
        xml_path = self._build_pkg_dir(xml)
        root = ET.fromstring(xml.encode("utf-8"))
        step = BuildTypeExportStep(
            _config(evaluate_conditions=evaluate_conditions), formatter=mock.Mock()
        )
        with mock.patch.dict(os.environ, env, clear=True):
            return step.perform_check(root, str(xml_path))

    def test_inactive_build_type_treated_as_missing(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <export>
    <build_type condition="$ROS_DISTRO == 'humble'">ament_cmake</build_type>
  </export>
</package>
"""
        result = self._run(xml, {"ROS_DISTRO": "jazzy"})
        self.assertFalse(result.valid)
        self.assertTrue(
            any("build_type" in e or "Incorrect" in e for e in result.errors)
        )

    def test_active_build_type_recognized(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <export>
    <build_type condition="$ROS_DISTRO == 'humble'">ament_cmake</build_type>
  </export>
</package>
"""
        result = self._run(xml, {"ROS_DISTRO": "humble"})
        self.assertTrue(result.valid)

    def test_ignore_conditions_falls_back_to_first(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <export>
    <build_type condition="$ROS_DISTRO == 'humble'">ament_cmake</build_type>
  </export>
</package>
"""
        result = self._run(xml, {"ROS_DISTRO": "jazzy"}, evaluate_conditions=False)
        self.assertTrue(result.valid)


class TestDependencyQueriesPreserveAttributesThroughDeepcopy(unittest.TestCase):
    """Regression: structural-check deepcopy paths must preserve condition attrs."""

    def test_condition_attribute_survives_deepcopy(self):
        from copy import deepcopy

        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <depend condition="$ROS_DISTRO == 'jazzy'">foo</depend>
</package>
"""
        root = ET.fromstring(xml.encode("utf-8"))
        copy = deepcopy(root)
        depend = copy.find("depend")
        self.assertEqual(depend.get("condition"), "$ROS_DISTRO == 'jazzy'")


if __name__ == "__main__":
    unittest.main()
