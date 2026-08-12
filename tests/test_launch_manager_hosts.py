"""Deriving what each launch-manager host runs, from the configuration and the launch trees."""

import os
import tempfile
import unittest
from unittest import mock

from package_xml_validation.helpers.launch_manager import derive


class LaunchManagerTestCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = self._tmp.name
        self.prefix = os.path.join(self.root, "install")
        self._env = mock.patch.dict(os.environ, {"AMENT_PREFIX_PATH": self.prefix})
        self._env.start()
        # The shared component library exists but is empty unless a test fills it, which is
        # the normal case: most components are defined by the repository itself.
        os.makedirs(
            os.path.join(self.prefix, "share", "launch_manager_common_components")
        )

    def tearDown(self):
        self._env.stop()
        self._tmp.cleanup()

    def write(self, path, text):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        return path

    def package(self, name):
        directory = os.path.join(self.root, name)
        self.write(
            os.path.join(directory, "package.xml"),
            f'<package format="3"><name>{name}</name></package>',
        )
        return directory

    def config(self, package_dir, components, name="default.yaml"):
        lines = ["components:"]
        for component, hosts in components:
            lines.append(f"  - name: {component}")
            if isinstance(hosts, str):
                lines.append(f"    hosts: {hosts}")
            elif hosts:
                lines.append("    hosts:")
                lines.extend(f"      - {one}" for one in hosts)
        self.write(
            os.path.join(package_dir, "launch_manager_configs", name),
            "\n".join(lines) + "\n",
        )

    def component(self, package_dir, name, launches, relative=""):
        """A component that starts `<package>/launch/<package>.launch.yaml`."""
        self.write(
            os.path.join(
                package_dir, "launch_manager_components", relative, f"{name}.yaml"
            ),
            f"launch:\n  package: {launches}\n  launch_file: {launches}.launch.yaml\n",
        )

    def installed_launch_file(self, package, names):
        """An installed package whose launch file names other packages."""
        body = "".join(f"  - node:\n      pkg: {one}\n" for one in names)
        self.write(
            os.path.join(
                self.prefix, "share", package, "launch", f"{package}.launch.yaml"
            ),
            f"launch:\n{body}",
        )

    def dirs(self):
        return sorted(
            os.path.join(self.root, one)
            for one in os.listdir(self.root)
            if os.path.isfile(os.path.join(self.root, one, "package.xml"))
        )


class TestWhatAHostRuns(LaunchManagerTestCase):
    def test_it_follows_the_component_into_the_launch_tree(self):
        """The point of the derivation: not the package the component names, but everything
        the launch file behind it pulls in."""
        self.installed_launch_file("driver", ["deep_dependency"])
        app = self.package("app")
        self.component(app, "a_driver", "driver")
        self.config(app, [("a_driver", "gripper")])

        found = derive(self.dirs())

        self.assertTrue(found.complete, found.problems + found.unresolved)
        self.assertEqual({"driver", "deep_dependency"}, found.packages["gripper"])

    def test_a_component_in_a_subdirectory_is_found(self):
        """Components are routinely grouped, e.g. into a firmware_drivers/ folder."""
        self.installed_launch_file("driver", [])
        app = self.package("app")
        self.component(app, "a_driver", "driver", relative="firmware_drivers")
        self.config(app, [("a_driver", "gripper")])

        found = derive(self.dirs())

        self.assertTrue(found.complete, found.problems + found.unresolved)
        self.assertEqual({"driver"}, found.packages["gripper"])

    def test_a_component_from_the_shared_library_is_found(self):
        """Some components are defined once and installed for every robot."""
        self.installed_launch_file("driver", [])
        self.write(
            os.path.join(
                self.prefix,
                "share",
                "launch_manager_common_components",
                "launch_manager_components",
                "shared_thing.yaml",
            ),
            "launch:\n  package: driver\n  launch_file: driver.launch.yaml\n",
        )
        app = self.package("app")
        self.config(app, [("shared_thing", "main")])

        found = derive(self.dirs())

        self.assertTrue(found.complete, found.problems + found.unresolved)
        self.assertEqual({"driver"}, found.packages["main"])

    def test_the_repository_overrides_a_shared_component(self):
        self.installed_launch_file("shared_driver", [])
        self.installed_launch_file("local_driver", [])
        self.write(
            os.path.join(
                self.prefix,
                "share",
                "launch_manager_common_components",
                "thing.yaml",
            ),
            "launch:\n  package: shared_driver\n  launch_file: shared_driver.launch.yaml\n",
        )
        app = self.package("app")
        self.component(app, "thing", "local_driver")
        self.config(app, [("thing", "main")])

        found = derive(self.dirs())

        self.assertEqual({"local_driver"}, found.packages["main"])

    def test_a_component_on_two_hosts_counts_for_both(self):
        self.installed_launch_file("driver", [])
        app = self.package("app")
        self.component(app, "a_driver", "driver")
        self.config(app, [("a_driver", ["main", "gripper"])])

        found = derive(self.dirs())

        self.assertEqual({"driver"}, found.packages["main"])
        self.assertEqual({"driver"}, found.packages["gripper"])

    def test_every_configuration_counts(self):
        """A robot may be started with any of them, so a package only one configuration needs
        is still a package the host has to have installed."""
        self.installed_launch_file("everyday", [])
        self.installed_launch_file("special", [])
        app = self.package("app")
        self.component(app, "everyday_thing", "everyday")
        self.component(app, "special_thing", "special")
        self.config(app, [("everyday_thing", "main")], name="default.yaml")
        self.config(app, [("special_thing", "main")], name="maze.yaml")

        found = derive(self.dirs())

        self.assertEqual({"everyday", "special"}, found.packages["main"])


class TestTheValidatorDelegates(LaunchManagerTestCase):
    """The configuration package's launch files name what every host runs; each install set
    declares its own share. Without the allowance every reference reads as missing here."""

    def install_set(self, name, host, deps):
        body = "".join(f"<exec_depend>{one}</exec_depend>" for one in deps)
        self.write(
            os.path.join(self.root, name, "package.xml"),
            f'<package format="3"><name>{name}</name>{body}'
            f"<export><launch_manager_host>{host}</launch_manager_host></export></package>",
        )

    def setUp(self):
        super().setUp()
        self.installed_launch_file("driver", [])
        self.app = self.package("app")
        self.write(
            os.path.join(self.app, "launch", "app.launch.yaml"),
            "launch:\n  - node:\n      pkg: driver\n",
        )
        self.component(self.app, "a_driver", "driver")
        self.config(self.app, [("a_driver", "gripper")])

    def missing_for_app(self):
        from package_xml_validation.package_xml_validator import PackageXmlValidator

        validator = PackageXmlValidator(check_only=True, check_rosdeps=False)
        # The exact logger: get_logger sets propagate=False, so a parent name sees nothing.
        with self.assertLogs("package_xml_validation.package_xml_validator") as logs:
            validator.check_and_format([self.root])
        return [
            one
            for one in logs.output
            if "Missing <exec_depend>" in one and "app/package.xml" in one
        ]

    def test_a_reference_an_install_set_declares_is_not_missing_here(self):
        self.install_set("app_gripper", "gripper", ["driver"])

        self.assertEqual([], self.missing_for_app())

    def test_a_reference_nobody_declares_still_is(self):
        self.install_set("app_gripper", "gripper", [])

        self.assertEqual(1, len(self.missing_for_app()))


class TestWhenItCannotAnswer(LaunchManagerTestCase):
    """An incomplete derivation must say so. Callers fill manifests from this."""

    def test_an_unreadable_component_is_named_and_marks_it_incomplete(self):
        app = self.package("app")
        self.config(app, [("nobody_defines_this", "main")])

        found = derive(self.dirs())

        self.assertEqual(["nobody_defines_this"], found.unresolved)
        self.assertFalse(found.complete)

    def test_two_configuration_packages_derive_nothing(self):
        """Which one describes the hosts is not something to guess at."""
        for name in ("app", "other"):
            package = self.package(name)
            self.config(package, [("a_driver", "main")])

        found = derive(self.dirs())

        self.assertEqual({}, found.packages)
        self.assertFalse(found.complete)
        self.assertIn("cannot tell which", " ".join(found.problems))

    def test_no_configuration_package_derives_nothing(self):
        self.package("app")

        found = derive(self.dirs())

        self.assertEqual({}, found.packages)
        self.assertFalse(found.complete)

    def test_a_missing_shared_library_marks_it_incomplete(self):
        """Without a built workspace the shared components cannot be read, so the sets are
        short by an unknown amount - which must not read as a clean result."""
        self.installed_launch_file("driver", [])
        app = self.package("app")
        self.component(app, "a_driver", "driver")
        self.config(app, [("a_driver", "main")])

        with mock.patch.dict(os.environ, {"AMENT_PREFIX_PATH": ""}):
            found = derive(self.dirs())

        self.assertFalse(found.complete)
        self.assertIn("launch_manager_common_components", " ".join(found.problems))

    def test_an_unparsable_configuration_is_reported(self):
        app = self.package("app")
        self.write(
            os.path.join(app, "launch_manager_configs", "default.yaml"),
            "components:\n  - [unclosed\n",
        )

        found = derive(self.dirs())

        self.assertFalse(found.complete)
        self.assertIn("could not be read", " ".join(found.problems))
