"""The launch-tree check: does everything this tree reaches exist, and is it declared?"""

import contextlib
import io
import os
import tempfile
import unittest
from unittest import mock

from package_xml_validation.check_launch_tree import (
    DEAD_INCLUDE,
    FATAL,
    LISTED_UNREAD,
    NOT_INSTALLED,
    UNDECLARED,
    WARNING,
    Scope,
    check,
    main,
    package_dirs_under,
    workspace_is_available,
)


def manifest(name, deps=(), ignore=()):
    body = "".join(f"<exec_depend>{one}</exec_depend>" for one in deps)
    if ignore:
        body += f"<!-- validator:ignore {' '.join(ignore)} -->"
    return f'<package format="3"><name>{name}</name>{body}</package>'


def of_kind(findings, kind):
    return [one for one in findings if one.kind == kind]


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

    def write(self, path, text):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        return path

    def install(self, package, relative, text="", deps=()):
        """A package in the install space, manifest included - the same as a real one."""
        share = os.path.join(self.prefix, "share", package)
        self.write(os.path.join(share, "package.xml"), manifest(package, deps))
        return self.write(os.path.join(share, relative), text)

    def write_into(self, directory, relative, text):
        """Another file in a package that already exists."""
        return self.write(os.path.join(directory, relative), text)

    def package(self, name, relative, text, under="", deps=(), ignore=()):
        """A source package: a `package.xml` plus one file, at `<root>/<under>/<name>`."""
        directory = os.path.join(self.root, under, name)
        self.write(os.path.join(directory, relative), text)
        self.write(
            os.path.join(directory, "package.xml"),
            manifest(os.path.basename(name), deps, ignore),
        )
        return directory


class TestWhatItFinds(LaunchTreeTestCase):
    def test_a_reference_the_manifest_does_not_declare(self):
        pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: some_driver\n",
        )

        _, findings = check(pkg)

        self.assertEqual(1, len(of_kind(findings, UNDECLARED)))
        self.assertEqual("some_driver", findings[0].package)
        self.assertEqual(FATAL, findings[0].severity)

    def test_a_declared_reference_is_not_a_manifest_finding(self):
        """Still a warning that it is not installed - that part is the workspace's business."""
        pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: some_driver\n",
            deps=["some_driver"],
        )

        _, findings = check(pkg)

        self.assertEqual([], of_kind(findings, UNDECLARED))
        self.assertEqual([WARNING], [one.severity for one in findings])

    def test_a_package_naming_itself_is_not_a_finding(self):
        pkg = self.package(
            "app", "launch/app.launch.yaml", "launch:\n  - node:\n      pkg: app\n"
        )

        _, findings = check(pkg)

        self.assertEqual([], findings)

    def test_a_package_that_is_not_installed_only_warns(self):
        """A CI runner may simply not have built it, so nothing in the checkout is wrong."""
        pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: absent_driver\n",
            deps=["absent_driver"],
        )

        _, findings = check(pkg)

        self.assertEqual(1, len(of_kind(findings, NOT_INSTALLED)))
        self.assertEqual(WARNING, findings[0].severity)

    def test_an_include_that_leads_nowhere(self):
        """A different fault: the package is there, the launch file it names is not."""
        self.install("driver", "launch/something_else.launch.yaml", "launch: []\n")
        pkg = self.package(
            "app",
            "launch_manager_components/broken.yaml",
            "launch:\n  package: driver\n  launch_file: not_shipped.launch.yaml\n",
            deps=["driver"],
        )

        _, findings = check(pkg)

        dead = of_kind(findings, DEAD_INCLUDE)
        self.assertEqual(1, len(dead))
        self.assertEqual("not_shipped.launch.yaml", dead[0].launch_file)
        self.assertEqual(FATAL, dead[0].severity)

    def test_a_missing_package_is_not_also_reported_as_a_dead_include(self):
        """One fault, one line: the include fails *because* the package is absent."""
        pkg = self.package(
            "app",
            "launch_manager_components/one.yaml",
            "launch:\n  package: absent\n  launch_file: x.launch.yaml\n",
            deps=["absent"],
        )

        _, findings = check(pkg)

        self.assertEqual([NOT_INSTALLED], [one.kind for one in findings])

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
            deps=["aggregator"],
        )

        _, findings = check(pkg)

        self.assertIn("absent_from_everywhere", [one.package for one in findings])

    def test_a_name_still_holding_a_substitution_is_neither(self):
        """It names no package, so it is neither a missing package nor a package that failed
        to ship a file - it is an include nobody could follow."""
        pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - include:\n"
            '      file: "$(find-pkg-share $(var undeclared)_sim)/launch/x.launch.yaml"\n',
        )

        walk, findings = check(pkg)

        self.assertEqual([], findings)
        self.assertEqual(1, len(walk.unresolved))

    def test_a_healthy_package_reports_nothing(self):
        self.install("driver", "launch/driver.launch.yaml", "launch: []\n")
        pkg = self.package(
            "app",
            "launch_manager_components/fine.yaml",
            "launch:\n  package: driver\n  launch_file: driver.launch.yaml\n",
            deps=["driver"],
        )

        _, findings = check(pkg)

        self.assertEqual([], findings)

    def test_a_config_key_merely_ending_in_package_is_not_a_reference(self):
        """`launch/` holds parameter files as well as launch files, and both are scanned. The
        anchored key is what keeps `plugin_package:` and friends out of a fatal finding."""
        pkg = self.package(
            "app",
            "launch/params.yaml",
            "some_node:\n  ros__parameters:\n    plugin_package: not_a_dependency\n",
        )

        _, findings = check(pkg)

        self.assertEqual([], findings)

    def test_a_bare_package_key_in_a_config_file_is_read_as_a_reference(self):
        """A known limitation, written down rather than implied away: nothing distinguishes a
        parameter file under `launch/` from a component definition, so a `package:` key in one
        is a fatal finding. No file in hector/src is shaped this way; if one appears, the fix
        is `<!-- validator:ignore -->` or `--ignore`."""
        pkg = self.package(
            "app",
            "launch/params.yaml",
            "some_node:\n  ros__parameters:\n    package: not_a_dependency\n",
        )

        _, findings = check(pkg)

        undeclared = of_kind(findings, UNDECLARED)
        self.assertEqual(["not_a_dependency"], [one.package for one in undeclared])
        self.assertEqual(FATAL, undeclared[0].severity)

    def test_a_file_with_no_manifest_above_it_is_not_graded(self):
        """There is nothing to hold responsible, so the reference is left alone rather than
        blamed on the package that reached it."""
        orphan = os.path.join(self.prefix, "share", "orphan")
        self.write(
            os.path.join(orphan, "launch", "x.launch.yaml"),
            "launch:\n  - node:\n      pkg: named_by_an_orphan\n",
        )
        pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            'launch:\n  - include:\n      file: "$(find-pkg-share orphan)/launch/x.launch.yaml"\n',
            deps=["orphan"],
        )

        _, findings = check(pkg)

        self.assertEqual([], of_kind(findings, UNDECLARED))
        self.assertNotIn(FATAL, [one.severity for one in findings])

    def test_a_package_named_twice_is_one_finding(self):
        """A manifest entry is missing once, however many nodes noticed."""
        pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n"
            "  - node:\n      pkg: some_driver\n"
            "  - node:\n      pkg: some_driver\n",
        )

        _, findings = check(pkg)

        self.assertEqual(1, len(of_kind(findings, UNDECLARED)))

    def test_a_dead_include_with_no_manifest_above_it_only_warns(self):
        """Same rule as everywhere else: no manifest to hold responsible, no failure."""
        self.write(
            os.path.join(self.prefix, "share", "orphan", "launch", "x.launch.yaml"),
            'launch:\n  - include:\n      file: "$(find-pkg-share driver)/launch/gone.launch.yaml"\n',
        )
        self.install("driver", "launch/a_different_file.launch.yaml", "launch: []\n")
        pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            'launch:\n  - include:\n      file: "$(find-pkg-share orphan)/launch/x.launch.yaml"\n',
            deps=["orphan"],
        )

        _, findings = check(pkg)

        dead = of_kind(findings, DEAD_INCLUDE)
        self.assertEqual(1, len(dead))
        self.assertEqual(WARNING, dead[0].severity)

    def test_a_manifest_that_does_not_parse_only_warns(self):
        """Whether it is valid XML at all is `format-package-xml`'s question, and it fails the
        commit on its own. This one has nothing to grade against, so it does not pile on."""
        pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: some_driver\n",
        )
        self.write(os.path.join(pkg, "package.xml"), "<package><name>app</name>")

        _, findings = check(pkg)

        self.assertNotIn(FATAL, [one.severity for one in findings])

    def test_an_ignored_dependency_is_left_out(self):
        pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: some_driver\n",
            ignore=["some_driver"],
        )

        _, findings = check(pkg)

        self.assertEqual([], findings)


class TestWhoIsHeldResponsible(LaunchTreeTestCase):
    """A finding is only fatal when the package that owns the offending file is fixable."""

    def reaching_an_installed_package(self, **package_kwargs):
        self.install(
            "aggregator",
            "launch/pipeline.launch.yaml",
            "launch:\n  - node:\n      pkg: undeclared_by_aggregator\n",
        )
        return self.package(
            "app",
            "launch_manager_components/detection.yaml",
            "launch:\n  package: aggregator\n  launch_file: pipeline.launch.yaml\n",
            deps=["aggregator"],
            **package_kwargs,
        )

    def test_a_binary_only_install_only_warns(self):
        """Nobody committing here can edit /opt, so failing on it would just block the repo."""
        pkg = self.reaching_an_installed_package()

        _, findings = check(pkg)

        undeclared = of_kind(findings, UNDECLARED)
        self.assertEqual(1, len(undeclared))
        self.assertEqual(WARNING, undeclared[0].severity)

    def test_fatal_under_makes_an_install_prefix_ours(self):
        """The CI shape: the pipeline installs the packages itself, so they are its to fix."""
        pkg = self.reaching_an_installed_package()
        scope = Scope.of([pkg], fatal_under=(self.prefix,))

        _, findings = check(pkg, scope=scope)

        self.assertEqual(FATAL, of_kind(findings, UNDECLARED)[0].severity)

    def test_the_checkouts_manifest_wins_over_the_installed_copy(self):
        """Includes resolve through find-pkg-share, so the file read is the installed one. The
        manifest to grade against is still the one in the checkout."""
        pkg = self.reaching_an_installed_package()
        source = self.package(
            "aggregator",
            "launch/pipeline.launch.yaml",
            "launch:\n  - node:\n      pkg: undeclared_by_aggregator\n",
        )
        scope = Scope.of([self.root])

        _, findings = check(pkg, scope=scope)

        undeclared = of_kind(findings, UNDECLARED)
        self.assertEqual(1, len(undeclared))
        self.assertEqual(os.path.join(source, "package.xml"), undeclared[0].owner)
        self.assertEqual(FATAL, undeclared[0].severity)


class TestFindingThePackagesToCheck(LaunchTreeTestCase):
    """The hook is handed a repository root, not a package directory."""

    def test_a_repository_root_finds_the_packages_below_it(self):
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

        with mock.patch("sys.argv", ["check-launch-tree", self.root]):
            with self.assertRaises(SystemExit) as exit:
                main()
        self.assertEqual(1, exit.exception.code)

    def test_a_directory_with_no_packages_says_so(self):
        with mock.patch("sys.argv", ["check-launch-tree", self.root]):
            main()  # must not raise SystemExit


class TestWithoutAWorkspace(LaunchTreeTestCase):
    """AMENT_PREFIX_PATH decides how far the walk reaches, not whether it runs."""

    def setUp(self):
        super().setUp()
        self.pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: some_driver\n",
        )
        self.empty = mock.patch.dict(os.environ, {"AMENT_PREFIX_PATH": ""})

    def test_it_knows_when_there_is_nothing_to_resolve_against(self):
        with self.empty:
            self.assertFalse(workspace_is_available())

    def test_nothing_is_reported_as_missing(self):
        """Every package would read as missing, which says nothing about the checkout."""
        with self.empty:
            _, findings = check(self.pkg)

        self.assertEqual([], of_kind(findings, NOT_INSTALLED))

    def test_the_manifest_check_still_runs(self):
        """It never needed a workspace, and it is the reason this can run in CI at all."""
        with self.empty:
            with mock.patch("sys.argv", ["check-launch-tree", self.pkg]):
                with self.assertRaises(SystemExit) as exit:
                    main()
        self.assertEqual(1, exit.exception.code)


class TestHowItFails(LaunchTreeTestCase):
    def setUp(self):
        super().setUp()
        self.pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: some_driver\n",
        )

    def test_a_manifest_finding_fails_the_run(self):
        with mock.patch("sys.argv", ["check-launch-tree", self.pkg]):
            with self.assertRaises(SystemExit) as exit:
                main()
        self.assertEqual(1, exit.exception.code)

    def test_warn_only_reports_the_same_thing_and_passes(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            with mock.patch("sys.argv", ["check-launch-tree", "--warn-only", self.pkg]):
                main()  # must not raise SystemExit

        self.assertIn("some_driver is not declared", out.getvalue())

    def test_ignore_drops_the_finding_altogether(self):
        with mock.patch(
            "sys.argv", ["check-launch-tree", "--ignore", "some_driver", self.pkg]
        ):
            main()  # must not raise SystemExit

    def test_a_clean_package_passes(self):
        clean = self.package("clean", "launch/x.launch.yaml", "launch: []\n")
        with mock.patch("sys.argv", ["check-launch-tree", clean]):
            main()


class TestWhatItPrints(LaunchTreeTestCase):
    """The report is the whole product of this tool, so its lines are worth asserting."""

    def report_for(self, package_dir, *argv):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            with mock.patch("sys.argv", ["check-launch-tree", *argv, package_dir]):
                with contextlib.suppress(SystemExit):
                    main()
        return out.getvalue()

    def test_a_clean_package_prints_nothing(self):
        """`verbose: true` shows everything this prints, so a clean run has to be silent."""
        clean = self.package("clean", "launch/x.launch.yaml", "launch: []\n")

        self.assertEqual("", self.report_for(clean))

    def test_a_finding_carries_its_severity_the_file_and_the_line(self):
        self.install("driver", "launch/something_else.launch.yaml", "launch: []\n")
        pkg = self.package(
            "app",
            "launch_manager_components/broken.yaml",
            "launch:\n  package: driver\n  launch_file: not_shipped.launch.yaml\n",
            deps=["driver"],
        )

        report = self.report_for(pkg)

        self.assertIn("error", report)
        self.assertIn("driver does not ship not_shipped.launch.yaml", report)
        self.assertIn("launch_manager_components/broken.yaml:2", report)

    def test_one_package_that_is_both_undeclared_and_absent_is_one_finding(self):
        pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: absent_driver\n",
        )

        report = self.report_for(pkg)

        self.assertIn(
            "is not declared in package.xml (and is not installed here)", report
        )
        self.assertNotIn("is not installed in this workspace", report)

    def test_a_package_that_is_only_absent_says_so_and_passes(self):
        """The line a CI runner without a built workspace will see most of."""
        pkg = self.package(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: absent_driver\n",
            deps=["absent_driver"],
        )

        report = self.report_for(pkg)

        self.assertIn("warning", report)
        self.assertIn("absent_driver is not installed in this workspace", report)

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
            deps=["aggregator"],
        )

        report = self.report_for(pkg)

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
        self.pkg = self.package(
            "scenario", "launch/robot.launch.yaml", self.SEED, deps=["telemax_sim"]
        )

    def test_the_default_leads_somewhere_that_is_not_installed(self):
        """`athena_sim` is not here, so the include is reported rather than guessed at."""
        walk, _ = check(self.pkg)

        self.assertEqual(1, len(walk.unresolved))
        self.assertIn("athena_sim", walk.unresolved[0].package)

    def test_a_supplied_value_follows_the_other_branch(self):
        """With robot=telemax the include resolves and what the file behind it names is
        checked - against telemax_sim's own manifest."""
        walk, findings = check(self.pkg, args={"robot": "telemax"})

        self.assertEqual([], walk.unresolved)
        self.assertIn("telemax_only", [one.package for one in findings])

    def test_the_cli_passes_arg_name_value_through(self):
        with mock.patch(
            "sys.argv",
            [
                "check-launch-tree",
                "--fatal-under",
                self.prefix,
                "--arg",
                "robot=telemax",
                self.pkg,
            ],
        ):
            with self.assertRaises(SystemExit) as exit:
                main()
        self.assertEqual(1, exit.exception.code)


class TestOneFindingIsReportedOnce(LaunchTreeTestCase):
    """Seed directories share one walk, so a file both reach is read once."""

    def setUp(self):
        super().setUp()
        self.install("driver", "launch/a_different_file.launch.yaml", "launch: []\n")
        self.install(
            "common",
            "launch/shared.launch.yaml",
            "launch:\n  - include:\n"
            '      file: "$(find-pkg-share driver)/launch/gone.launch.yaml"\n',
            deps=["driver"],
        )

        reaches_shared = (
            "launch:\n  - include:\n"
            '      file: "$(find-pkg-share common)/launch/shared.launch.yaml"\n'
        )
        self.pkg = self.package(
            "app", "launch/a.launch.yaml", reaches_shared, deps=["common"]
        )
        self.write_into(self.pkg, "launch_manager_components/b.yaml", reaches_shared)

    def test_the_shared_file_is_read_once(self):
        walk, _ = check(self.pkg)

        read_shared = [f for f in walk.visited if f.endswith("shared.launch.yaml")]
        self.assertEqual(1, len(read_shared))

    def test_its_dead_include_is_reported_once(self):
        _, findings = check(self.pkg)

        self.assertEqual(1, len(of_kind(findings, DEAD_INCLUDE)))
