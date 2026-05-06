"""Tests for the second batch of REP-149 conformance fixes.

Covers:
  - LaunchDependencyStep is condition-aware (inactive deps don't satisfy launch refs)
  - BuildTypeExportStep applies REP-149's last-wins rule + warns on multi-active
  - ManifestSchemaStep rejects non-<package> root elements
  - MemberOfGroupStep ignores inactive <member_of_group> entries
  - RosidlInterfaceRuntimeStep requires <exec_depend>rosidl_default_runtime</…>
"""

from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import lxml.etree as ET

from package_xml_validation.helpers.validation_steps import (
    BuildTypeExportStep,
    LaunchDependencyStep,
    ManifestSchemaStep,
    MemberOfGroupStep,
    RosidlInterfaceRuntimeStep,
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
        evaluate_conditions=True,
    )
    base.update(overrides)
    return ValidationConfig(**base)


def _parse(xml: str):
    return ET.fromstring(xml.encode("utf-8"))


def _msg_pkg_dir() -> Path:
    """A CMake package whose CMakeLists triggers is_msg_pkg=True."""
    tmp = Path(tempfile.mkdtemp(prefix="rep149_followup_"))
    (tmp / "CMakeLists.txt").write_text(
        "project(demo)\nrosidl_generate_interfaces(${PROJECT_NAME} msg/Foo.msg)\n"
    )
    return tmp


class TestRootTagCheck(unittest.TestCase):
    """ManifestSchemaStep must reject non-<package> roots."""

    def setUp(self):
        tmp = tempfile.TemporaryDirectory(prefix="root_tag_")
        self.addCleanup(tmp.cleanup)
        self.fake_xml = os.path.join(tmp.name, "demo", "package.xml")

    def test_wrong_root_tag_errors(self):
        xml = """<?xml version="1.0"?>
<wrong_root format="3">
  <name>demo</name>
</wrong_root>
"""
        result = ManifestSchemaStep(_config()).perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)
        self.assertTrue(any("<wrong_root>" in e for e in result.errors))

    def test_correct_root_tag_passes_root_check(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <version>0.0.1</version>
  <maintainer email="a@example.com">A</maintainer>
</package>
"""
        result = ManifestSchemaStep(_config()).perform_check(_parse(xml), self.fake_xml)
        # No root-tag error; other rules independent.
        self.assertFalse(any("REP-149 requires <package>" in e for e in result.errors))


class TestBuildTypeLastWins(unittest.TestCase):
    """REP-149: when multiple <build_type> are active, the last one wins."""

    def _build_pkg_dir(self, xml: str) -> Path:
        tmp = Path(tempfile.mkdtemp(prefix="last_wins_"))
        (tmp / "CMakeLists.txt").write_text("project(demo)\n")
        (tmp / "package.xml").write_text(xml, encoding="utf-8")
        return tmp

    def test_last_wins_when_two_unconditional_actives(self):
        # Two unconditional <build_type>: spec is silent on which wins, but
        # we treat all as active and pick the last (matches the spec's
        # last-wins rule when conditions all evaluate True).
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <export>
    <build_type>ament_python</build_type>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
        pkg = self._build_pkg_dir(xml)
        root = _parse(xml)
        result = BuildTypeExportStep(_config(), formatter=mock.Mock()).perform_check(
            root, str(pkg / "package.xml")
        )
        # Last is ament_cmake → matches CMAKE_PKG → valid.
        self.assertTrue(result.valid)
        # And we warn because two are active.
        self.assertTrue(
            any("multiple active <build_type>" in w for w in result.warnings)
        )

    def test_last_wins_with_conditions(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <export>
    <build_type condition="$ROS_DISTRO == 'jazzy'">ament_python</build_type>
    <build_type condition="$ROS_DISTRO == 'jazzy'">ament_cmake</build_type>
  </export>
</package>
"""
        pkg = self._build_pkg_dir(xml)
        root = _parse(xml)
        with mock.patch.dict(os.environ, {"ROS_DISTRO": "jazzy"}, clear=True):
            result = BuildTypeExportStep(
                _config(), formatter=mock.Mock()
            ).perform_check(root, str(pkg / "package.xml"))
        self.assertTrue(result.valid)
        self.assertTrue(
            any("multiple active <build_type>" in w for w in result.warnings)
        )

    def test_only_one_active_no_warning(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <export>
    <build_type condition="$ROS_DISTRO == 'humble'">ament_python</build_type>
    <build_type condition="$ROS_DISTRO == 'jazzy'">ament_cmake</build_type>
  </export>
</package>
"""
        pkg = self._build_pkg_dir(xml)
        root = _parse(xml)
        with mock.patch.dict(os.environ, {"ROS_DISTRO": "jazzy"}, clear=True):
            result = BuildTypeExportStep(
                _config(), formatter=mock.Mock()
            ).perform_check(root, str(pkg / "package.xml"))
        self.assertTrue(result.valid)
        self.assertFalse(
            any("multiple active <build_type>" in w for w in result.warnings)
        )


class TestMemberOfGroupConditionAware(unittest.TestCase):
    def _setup(self, xml: str, env: dict):
        pkg = _msg_pkg_dir()
        (pkg / "package.xml").write_text(xml, encoding="utf-8")
        root = _parse(xml)
        step = MemberOfGroupStep(_config(), formatter=mock.Mock())
        with mock.patch.dict(os.environ, env, clear=True):
            return step.perform_check(root, str(pkg / "package.xml"))

    def test_inactive_member_of_group_treated_as_missing(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo_msgs</name>
  <member_of_group condition="$ROS_DISTRO == 'humble'">rosidl_interface_packages</member_of_group>
</package>
"""
        result = self._setup(xml, {"ROS_DISTRO": "jazzy"})
        self.assertFalse(result.valid)
        self.assertTrue(any("member_of_group" in e for e in result.errors))

    def test_active_member_of_group_satisfies_rule(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo_msgs</name>
  <member_of_group condition="$ROS_DISTRO == 'humble'">rosidl_interface_packages</member_of_group>
</package>
"""
        result = self._setup(xml, {"ROS_DISTRO": "humble"})
        self.assertTrue(result.valid)

    def test_unconditional_member_of_group_satisfies_rule(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo_msgs</name>
  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
"""
        result = self._setup(xml, {})
        self.assertTrue(result.valid)


class TestLaunchDependencyConditionAware(unittest.TestCase):
    """An inactive manifest dep must not satisfy a real launch reference."""

    def _build_pkg(self, xml: str) -> Path:
        tmp = Path(tempfile.mkdtemp(prefix="launch_cond_"))
        (tmp / "CMakeLists.txt").write_text("project(demo)\n")
        launch = tmp / "launch"
        launch.mkdir()
        (launch / "demo.launch.py").write_text(
            "from launch_ros.actions import Node\n"
            "Node(package='other_pkg', executable='x')\n"
        )
        (tmp / "package.xml").write_text(xml, encoding="utf-8")
        return tmp

    def test_inactive_exec_depend_does_not_cover_launch_ref(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <exec_depend condition="$ROS_DISTRO == 'humble'">other_pkg</exec_depend>
</package>
"""
        pkg = self._build_pkg(xml)
        root = _parse(xml)
        step = LaunchDependencyStep(
            _config(),
            formatter=mock.Mock(),
            rosdep_validator=None,
            package_name="demo",
        )
        with mock.patch.dict(os.environ, {"ROS_DISTRO": "jazzy"}, clear=True):
            result = step.perform_check(root, str(pkg / "package.xml"))
        self.assertFalse(result.valid)
        self.assertTrue(any("other_pkg" in e for e in result.errors))

    def test_active_exec_depend_covers_launch_ref(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <exec_depend condition="$ROS_DISTRO == 'jazzy'">other_pkg</exec_depend>
</package>
"""
        pkg = self._build_pkg(xml)
        root = _parse(xml)
        step = LaunchDependencyStep(
            _config(),
            formatter=mock.Mock(),
            rosdep_validator=None,
            package_name="demo",
        )
        with mock.patch.dict(os.environ, {"ROS_DISTRO": "jazzy"}, clear=True):
            result = step.perform_check(root, str(pkg / "package.xml"))
        self.assertTrue(result.valid)

    def test_ignore_conditions_falls_back(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <exec_depend condition="$ROS_DISTRO == 'humble'">other_pkg</exec_depend>
</package>
"""
        pkg = self._build_pkg(xml)
        root = _parse(xml)
        step = LaunchDependencyStep(
            _config(evaluate_conditions=False),
            formatter=mock.Mock(),
            rosdep_validator=None,
            package_name="demo",
        )
        with mock.patch.dict(os.environ, {"ROS_DISTRO": "jazzy"}, clear=True):
            result = step.perform_check(root, str(pkg / "package.xml"))
        self.assertTrue(result.valid)


class TestRosidlDefaultRuntimeStep(unittest.TestCase):
    def _setup(self, xml: str, **config):
        pkg = _msg_pkg_dir()
        (pkg / "package.xml").write_text(xml, encoding="utf-8")
        root = _parse(xml)
        formatter = mock.Mock()
        step = RosidlInterfaceRuntimeStep(_config(**config), formatter=formatter)
        return step.perform_check(root, str(pkg / "package.xml")), formatter, root

    def test_missing_runtime_errors_for_msg_pkg(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo_msgs</name>
</package>
"""
        result, _, _ = self._setup(xml)
        self.assertFalse(result.valid)
        self.assertTrue(any("rosidl_default_runtime" in e for e in result.errors))

    def test_present_runtime_passes(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo_msgs</name>
  <exec_depend>rosidl_default_runtime</exec_depend>
</package>
"""
        result, _, _ = self._setup(xml)
        self.assertTrue(result.valid)

    def test_unified_depend_satisfies_rule(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo_msgs</name>
  <depend>rosidl_default_runtime</depend>
</package>
"""
        result, _, _ = self._setup(xml)
        self.assertTrue(result.valid)

    def test_inactive_runtime_does_not_satisfy(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo_msgs</name>
  <exec_depend condition="$ROS_DISTRO == 'humble'">rosidl_default_runtime</exec_depend>
</package>
"""
        with mock.patch.dict(os.environ, {"ROS_DISTRO": "jazzy"}, clear=True):
            result, _, _ = self._setup(xml)
        self.assertFalse(result.valid)

    def test_auto_fill_inserts_runtime(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo_msgs</name>
</package>
"""
        result, formatter, root = self._setup(
            xml, check_only=False, auto_fill_missing_deps=True
        )
        self.assertTrue(result.changed)
        formatter.add_dependencies.assert_called_once_with(
            root, ["rosidl_default_runtime"], "exec_depend"
        )

    def test_non_msg_pkg_skipped(self):
        # Plain CMake (no rosidl_generate_interfaces in CMakeLists).
        tmp = Path(tempfile.mkdtemp(prefix="nonmsg_"))
        (tmp / "CMakeLists.txt").write_text("project(demo)\n")
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
</package>
"""
        (tmp / "package.xml").write_text(xml, encoding="utf-8")
        result = RosidlInterfaceRuntimeStep(
            _config(), formatter=mock.Mock()
        ).perform_check(_parse(xml), str(tmp / "package.xml"))
        self.assertTrue(result.valid)


if __name__ == "__main__":
    unittest.main()
