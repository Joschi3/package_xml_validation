"""The launch-tree check: does everything this tree reaches exist here?"""

import os
import tempfile
import unittest
from unittest import mock

from package_xml_validation.check_launch_tree import check, main, workspace_is_available


class LaunchTreeTestCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = self._tmp.name
        # A prefix that looks like an install space, so the real lookup can find things
        self.prefix = os.path.join(self.root, "install")
        self._env = mock.patch.dict(os.environ, {"AMENT_PREFIX_PATH": self.prefix})
        self._env.start()

    def tearDown(self):
        self._env.stop()
        self._tmp.cleanup()

    def install(self, package, relative, text=""):
        path = os.path.join(self.prefix, "share", package, relative)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        return path

    def package(self, name, relative, text):
        path = os.path.join(self.root, name, relative)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        return os.path.join(self.root, name)


class TestWhatItFinds(LaunchTreeTestCase):
    def test_a_package_that_is_not_installed(self):
        """The launch file names it and nothing here provides it."""
        pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: absent_driver\n",
        )

        _, missing, dead = check(pkg)

        self.assertEqual(["absent_driver"], missing)
        self.assertEqual([], dead)

    def test_an_include_that_leads_nowhere(self):
        """A different fault: the package is there, the launch file it names is not."""
        self.install("driver", "launch/something_else.launch.yaml", "launch: []\n")
        pkg = self.package(
            "app",
            "launch_manager_components/broken.yaml",
            "launch:\n  package: driver\n  launch_file: not_shipped.launch.yaml\n",
        )

        _, missing, dead = check(pkg)

        self.assertEqual([], missing)
        self.assertEqual(1, len(dead))
        self.assertIn("not_shipped.launch.yaml", dead[0].launch_file)

    def test_a_missing_package_is_not_also_reported_as_a_dead_include(self):
        """One fault, one line: the include fails *because* the package is absent."""
        pkg = self.package(
            "app",
            "launch_manager_components/one.yaml",
            "launch:\n  package: absent\n  launch_file: x.launch.yaml\n",
        )

        _, missing, dead = check(pkg)

        self.assertEqual(["absent"], missing)
        self.assertEqual([], dead)

    def test_it_follows_into_other_packages(self):
        """The reference that breaks a tree is often not in the repository being committed to."""
        self.install(
            "aggregator",
            "launch/pipeline.launch.yaml",
            "launch:\n  - node:\n      pkg: absent_from_everywhere\n",
        )
        pkg = self.package(
            "app",
            "launch_manager_components/detection.yaml",
            "launch:\n  package: aggregator\n  launch_file: pipeline.launch.yaml\n",
        )

        _, missing, _ = check(pkg)

        self.assertEqual(["absent_from_everywhere"], missing)

    def test_a_healthy_package_reports_nothing(self):
        self.install("driver", "launch/driver.launch.yaml", "launch: []\n")
        pkg = self.package(
            "app",
            "launch_manager_components/fine.yaml",
            "launch:\n  package: driver\n  launch_file: driver.launch.yaml\n",
        )

        _, missing, dead = check(pkg)

        self.assertEqual([], missing)
        self.assertEqual([], dead)


class TestItNeedsAWorkspaceAndSaysSo(LaunchTreeTestCase):
    """Without AMENT_PREFIX_PATH there is nothing to resolve against, so the check skips."""

    def test_it_reports_that_it_could_not_look(self):
        with mock.patch.dict(os.environ, {"AMENT_PREFIX_PATH": ""}):
            self.assertFalse(workspace_is_available())

    def test_it_does_not_fail_when_there_is_nothing_to_resolve_against(self):
        """Not even with --error: every package would read as missing."""
        pkg = self.package(
            "app", "launch/app.launch.yaml", "launch:\n  - node:\n      pkg: anything\n"
        )

        with mock.patch.dict(os.environ, {"AMENT_PREFIX_PATH": ""}):
            with mock.patch("sys.argv", ["check-launch-tree", "--error", pkg]):
                main()  # must not raise SystemExit


class TestHowItFails(LaunchTreeTestCase):
    """Warns by default so it can be introduced to a repository that already has findings."""

    def setUp(self):
        super().setUp()
        self.pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: absent_driver\n",
        )

    def test_findings_alone_do_not_fail_the_hook(self):
        with mock.patch("sys.argv", ["check-launch-tree", self.pkg]):
            main()

    def test_error_makes_findings_fatal(self):
        with mock.patch("sys.argv", ["check-launch-tree", "--error", self.pkg]):
            with self.assertRaises(SystemExit) as exit:
                main()
        self.assertEqual(1, exit.exception.code)

    def test_a_clean_package_passes_with_error(self):
        clean = self.package("clean", "launch/x.launch.yaml", "launch: []\n")
        with mock.patch("sys.argv", ["check-launch-tree", "--error", clean]):
            main()


class TestLaunchArguments(LaunchTreeTestCase):
    """A tree whose shape depends on an argument is only followed once a value is known."""

    SEED = (
        "launch:\n"
        "  - arg:\n"
        "      name: robot\n"
        "      default: athena\n"
        "  - include:\n"
        '      file: "$(find-pkg-share $(var robot)_sim)/launch/spawn.launch.yaml"\n'
    )

    def setUp(self):
        super().setUp()
        # Only the telemax half of the workspace exists
        self.install(
            "telemax_sim",
            "launch/spawn.launch.yaml",
            "launch:\n  - node:\n      pkg: telemax_only\n",
        )
        self.pkg = self.package("scenario", "launch/robot.launch.yaml", self.SEED)

    def test_the_default_leads_somewhere_that_is_not_installed(self):
        """`athena_sim` is not here, so the include is reported rather than guessed at."""
        walk, missing, _ = check(self.pkg)

        self.assertEqual(1, len(walk.unresolved))
        self.assertIn("athena_sim", walk.unresolved[0].package)
        self.assertNotIn("telemax_only", missing)

    def test_a_supplied_value_follows_the_other_branch(self):
        """With robot=telemax the include resolves and what the file behind it names is
        checked."""
        walk, missing, _ = check(self.pkg, args={"robot": "telemax"})

        self.assertEqual([], walk.unresolved)
        self.assertIn("telemax_only", missing)
