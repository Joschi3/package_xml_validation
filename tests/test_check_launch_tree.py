"""The launch-tree check: does everything this tree reaches exist here?"""

import contextlib
import io
import os
import tempfile
import unittest
from unittest import mock

from package_xml_validation.check_launch_tree import (
    LISTED_UNREAD,
    check,
    main,
    package_dirs_under,
    workspace_is_available,
)


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

    def write_into(self, directory, relative, text):
        """Another file in a package that already exists."""
        path = os.path.join(directory, relative)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        return path

    def package(self, name, relative, text, under=""):
        """A source package: a `package.xml` plus one file, at `<root>/<under>/<name>`."""
        directory = os.path.join(self.root, under, name)
        path = os.path.join(directory, relative)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        with open(os.path.join(directory, "package.xml"), "w", encoding="utf-8") as f:
            f.write(
                f'<package format="3"><name>{os.path.basename(name)}</name></package>'
            )
        return directory


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

    def test_a_name_still_holding_a_substitution_is_neither(self):
        """It names no package, so it is neither a missing package nor a package that failed
        to ship a file - it is an include nobody could follow, and was reported as
        `$(var undeclared)_sim does not ship ...`."""
        pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - include:\n"
            '      file: "$(find-pkg-share $(var undeclared)_sim)/launch/x.launch.yaml"\n',
        )

        walk, missing, dead = check(pkg)

        self.assertEqual([], missing)
        self.assertEqual([], dead)
        self.assertEqual(1, len(walk.unresolved))

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


class TestFindingThePackagesToCheck(LaunchTreeTestCase):
    """The hook is handed a repository root, not a package directory."""

    def test_a_repository_root_finds_the_packages_below_it(self):
        """`check` works one package at a time, so nothing under `src/` was ever read and the
        hook passed silently on every multi-package repository."""
        self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: absent_driver\n",
            under="src",
        )

        found = package_dirs_under([self.root])

        self.assertEqual([os.path.join(self.root, "src", "app")], found)

    def test_it_fails_on_a_finding_below_the_root(self):
        self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: absent_driver\n",
            under="src",
        )

        with mock.patch("sys.argv", ["check-launch-tree", "--error", self.root]):
            with self.assertRaises(SystemExit) as exit:
                main()
        self.assertEqual(1, exit.exception.code)

    def test_a_directory_with_no_packages_says_so(self):
        with mock.patch("sys.argv", ["check-launch-tree", "--error", self.root]):
            main()  # must not raise SystemExit


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


class TestWhatItPrints(LaunchTreeTestCase):
    """The report is the whole product of this tool, so its lines are worth asserting."""

    def report_for(self, package_dir):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            with mock.patch("sys.argv", ["check-launch-tree", package_dir]):
                main()
        return out.getvalue()

    def test_a_dead_include_names_the_package_the_file_and_the_line(self):
        self.install("driver", "launch/something_else.launch.yaml", "launch: []\n")
        pkg = self.package(
            "app",
            "launch_manager_components/broken.yaml",
            "launch:\n  package: driver\n  launch_file: not_shipped.launch.yaml\n",
        )

        report = self.report_for(pkg)

        self.assertIn("driver does not ship launch/not_shipped.launch.yaml", report)
        self.assertIn("launch_manager_components/broken.yaml:2", report)

    def test_a_file_outside_the_package_is_named_absolutely(self):
        """The reference that breaks a tree is often in another package entirely, where a
        relative path would be a chain of `../..`."""
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

        report = self.report_for(pkg)

        self.assertIn("absent_from_everywhere is not installed", report)
        self.assertIn(
            os.path.join(self.prefix, "share", "aggregator", "launch"), report
        )

    def test_unfollowable_includes_beyond_the_listed_few_are_counted(self):
        """A truncated list that does not say it is truncated reads as the whole story."""
        lines = ["launch:"]
        for i in range(LISTED_UNREAD + 3):
            lines.append("  - include:")
            lines.append(
                f'      file: "$(find-pkg-share $(var undeclared)_{i})/launch/x.launch.yaml"'
            )
        pkg = self.package("app", "launch/app.launch.yaml", "\n".join(lines) + "\n")

        report = self.report_for(pkg)

        self.assertIn(f"{LISTED_UNREAD + 3} include(s) could not be followed", report)
        self.assertIn("... and 3 more", report)


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

    def test_the_cli_passes_arg_name_value_through(self):
        """`--arg robot=telemax` reaches the walk, so the include resolves and the package
        behind it is found missing - which fails the run."""
        with mock.patch(
            "sys.argv",
            ["check-launch-tree", "--error", "--arg", "robot=telemax", self.pkg],
        ):
            with self.assertRaises(SystemExit) as exit:
                main()
        self.assertEqual(1, exit.exception.code)


class TestOneFindingIsReportedOnce(LaunchTreeTestCase):
    """Seed directories share one walk, so a file both reach is read once.

    Walking each directory separately re-read everything reachable from more than one of them,
    so a finding inside a shared launch file was reported once per directory that led to it.
    """

    def setUp(self):
        super().setUp()
        self.install("driver", "launch/a_different_file.launch.yaml", "launch: []\n")
        self.install(
            "common",
            "launch/shared.launch.yaml",
            "launch:\n  - include:\n"
            '      file: "$(find-pkg-share driver)/launch/gone.launch.yaml"\n',
        )

        reaches_shared = (
            "launch:\n  - include:\n"
            '      file: "$(find-pkg-share common)/launch/shared.launch.yaml"\n'
        )
        self.pkg = self.package("app", "launch/a.launch.yaml", reaches_shared)
        self.write_into(self.pkg, "launch_manager_components/b.yaml", reaches_shared)

    def test_the_shared_file_is_read_once(self):
        walk, _, _ = check(self.pkg)

        read_shared = [f for f in walk.visited if f.endswith("shared.launch.yaml")]
        self.assertEqual(1, len(read_shared))

    def test_its_dead_include_is_reported_once(self):
        _, _, dead = check(self.pkg)

        self.assertEqual(1, len(dead))
