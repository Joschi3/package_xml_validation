"""`HostDependencyStep`: an install set checked against what its host actually launches."""

import unittest
import unittest.mock

import lxml.etree as ET

from package_xml_validation.helpers.launch_manager import HostDependencies
from package_xml_validation.helpers.pkg_xml_formatter import PackageXmlFormatter
from package_xml_validation.helpers.validation_steps import (
    HostDependencyStep,
    ValidationConfig,
)


def config(**overrides):
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


def manifest(name, host=None, deps=()):
    body = "".join(f"<exec_depend>{one}</exec_depend>" for one in deps)
    export = (
        f"<export><build_type>ament_cmake</build_type>"
        f"<launch_manager_host>{host}</launch_manager_host></export>"
        if host
        else ""
    )
    return ET.fromstring(
        f'<package format="3"><name>{name}</name>{body}{export}</package>'.encode()
    )


def step(hosts, **overrides):
    return HostDependencyStep(
        config(**overrides),
        PackageXmlFormatter(),
        None,
        "install_set",
        hosts,
    )


def declared(root):
    return [one.text for one in root if one.tag == "exec_depend"]


class TestAnInstallSet(unittest.TestCase):
    HOSTS = HostDependencies(packages={"gripper": {"driver_a", "driver_b"}})

    def test_a_package_the_host_launches_and_it_does_not_declare_is_an_error(self):
        root = manifest("install_set", host="gripper", deps=["driver_a"])

        result = step(self.HOSTS, check_only=True).perform_check(root, "package.xml")

        self.assertFalse(result.valid)
        self.assertIn("driver_b", result.errors[0])

    def test_auto_fill_adds_it(self):
        root = manifest("install_set", host="gripper", deps=["driver_a"])

        result = step(self.HOSTS).perform_check(root, "package.xml")

        self.assertTrue(result.changed)
        self.assertEqual(["driver_a", "driver_b"], sorted(declared(result.root)))

    def test_a_declared_package_it_could_not_derive_is_left_alone(self):
        """Controller plugins and packages passed to a component as arguments are named
        nowhere the walk reaches. Removing them would uninstall something the robot needs, and
        reporting them would be noise."""
        root = manifest(
            "install_set",
            host="gripper",
            deps=["driver_a", "driver_b", "a_controller_plugin"],
        )

        result = step(self.HOSTS, check_only=True).perform_check(root, "package.xml")

        self.assertTrue(result.valid)
        self.assertEqual([], result.errors)
        self.assertIn("a_controller_plugin", declared(result.root))

    def test_a_package_with_no_host_export_is_not_this_step_s_business(self):
        root = manifest("ordinary", deps=[])

        result = step(self.HOSTS, check_only=True).perform_check(root, "package.xml")

        self.assertTrue(result.valid)
        self.assertEqual([], result.errors)

    def test_it_does_not_demand_a_dependency_on_itself(self):
        hosts = HostDependencies(packages={"gripper": {"install_set", "driver_a"}})
        root = manifest("install_set", host="gripper", deps=["driver_a"])

        result = step(hosts, check_only=True).perform_check(root, "package.xml")

        self.assertTrue(result.valid)


class TestWhenItCannotBeSure(unittest.TestCase):
    def test_a_host_no_configuration_mentions_is_an_error(self):
        """Otherwise a typo in the export silently switches the check off for that package."""
        hosts = HostDependencies(packages={"gripper": {"driver_a"}})
        root = manifest("install_set", host="grippr")

        result = step(hosts, check_only=True).perform_check(root, "package.xml")

        self.assertFalse(result.valid)
        self.assertIn("grippr", result.errors[0])
        self.assertIn("gripper", result.errors[0])

    def test_an_unknown_host_is_only_an_error_once_the_configuration_was_read(self):
        """When the derivation failed the host may well be in a configuration nobody could
        read, and saying it is not sends the reader to fix a manifest that is already right."""
        hosts = HostDependencies(
            packages={},
            problems=["2 packages hold a launch_manager_configs/ directory"],
        )
        root = manifest("install_set", host="gripper")

        result = step(hosts, check_only=True).perform_check(root, "package.xml")

        self.assertTrue(result.valid)
        self.assertEqual([], result.errors)
        self.assertIn("Cannot tell what host 'gripper' runs", result.warnings[0])

    def test_an_incomplete_derivation_never_fills(self):
        """The set is short by an unknown amount, so filling from it would write a manifest
        that looks complete and is not."""
        hosts = HostDependencies(
            packages={"gripper": {"driver_a"}}, unresolved=["a_component"]
        )
        root = manifest("install_set", host="gripper")

        result = step(hosts).perform_check(root, "package.xml")

        self.assertFalse(result.changed)
        self.assertEqual([], declared(result.root))
        self.assertIn("a_component", result.warnings[0])

    def test_a_derived_name_rosdep_cannot_resolve_is_not_written(self):
        """A component naming something that is not a package would otherwise put an
        unresolvable key into a manifest, and rosdep would fail on every machine."""
        hosts = HostDependencies(packages={"gripper": {"not_a_real_package"}})
        rosdep = unittest.mock.MagicMock()
        rosdep.check_rosdeps_and_local_pkgs.return_value = ["not_a_real_package"]
        under_test = HostDependencyStep(
            config(check_rosdeps=True),
            PackageXmlFormatter(),
            rosdep,
            "install_set",
            hosts,
        )
        root = manifest("install_set", host="gripper")

        result = under_test.perform_check(root, "package.xml")

        self.assertFalse(result.changed)
        self.assertEqual([], declared(result.root))
        self.assertIn("not_a_real_package", result.critical_errors[0])

    def test_an_incomplete_derivation_says_why(self):
        hosts = HostDependencies(
            packages={"gripper": {"driver_a"}},
            problems=["launch_manager_common_components is not installed"],
        )
        root = manifest("install_set", host="gripper")

        result = step(hosts).perform_check(root, "package.xml")

        self.assertFalse(result.changed)
        self.assertIn("not installed", result.warnings[0])
