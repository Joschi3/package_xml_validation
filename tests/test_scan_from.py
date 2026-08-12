"""Following a launch tree from a seed, rather than scanning a directory.

The property these tests protect is that the walk never overstates its coverage: an include it
cannot follow has to be reported, not skipped.
"""

import os
import tempfile
import unittest

from package_xml_validation.helpers.find_launch_dependencies import (
    scan_file_detailed,
    scan_from,
)


class Workspace:
    """A throwaway tree of launch files, with a share lookup that only knows what we put in it."""

    def __init__(self, root):
        self.root = root

    def write(self, package, relative, text):
        path = os.path.join(self.root, package, relative)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        return path

    def lookup(self, package):
        share = os.path.join(self.root, package)
        return share if os.path.isdir(share) else None


class ScanFromTestCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.ws = Workspace(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()


class TestFollowingIncludes(ScanFromTestCase):
    def test_it_reads_the_file_an_include_points_at(self):
        """A package this launch pulls in may name packages of its own."""
        self.ws.write(
            "downstream",
            "launch/child.launch.yaml",
            "launch:\n  - node:\n      pkg: only_named_downstream\n",
        )
        seed = self.ws.write(
            "upstream",
            "launch/parent.launch.yaml",
            'launch:\n  - include:\n      file: "$(find-pkg-share downstream)/launch/child.launch.yaml"\n',
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertIn("only_named_downstream", walk.packages)
        self.assertEqual(2, len(walk.visited))
        self.assertEqual([], walk.unresolved)

    def test_a_component_definition_is_an_include_too(self):
        """`package:` + `launch_file:` is how a component names what it starts."""
        self.ws.write(
            "aggregator",
            "launch/pipeline.launch.yaml",
            "launch:\n  - node:\n      pkg: a_detector\n",
        )
        seed = self.ws.write(
            "config",
            "components/detection.yaml",
            "launch:\n  package: aggregator\n  launch_file: pipeline.launch.yaml\n",
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertIn("a_detector", walk.packages)

    def test_a_leaf_reference_is_not_descended_into(self):
        """`pkg:` on a node has no launch file behind it, so nothing is descended into."""
        self.ws.write(
            "some_driver",
            "launch/unrelated.launch.yaml",
            "launch:\n  - node:\n      pkg: never_launched_by_us\n",
        )
        seed = self.ws.write(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: some_driver\n",
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertIn("some_driver", walk.packages)
        self.assertNotIn("never_launched_by_us", walk.packages)

    def test_a_cycle_terminates(self):
        a = self.ws.write(
            "a",
            "launch/a.launch.yaml",
            'launch:\n  - include:\n      file: "$(find-pkg-share b)/launch/b.launch.yaml"\n',
        )
        self.ws.write(
            "b",
            "launch/b.launch.yaml",
            'launch:\n  - include:\n      file: "$(find-pkg-share a)/launch/a.launch.yaml"\n',
        )

        walk = scan_from(a, share_lookup=self.ws.lookup)

        self.assertEqual(2, len(walk.visited))


class TestItNeverOverstatesWhatItRead(ScanFromTestCase):
    """A walk that skips what it cannot resolve reports full coverage of a partly read tree."""

    def test_a_computed_package_name_is_reported_not_skipped(self):
        """A package name assembled from a substitution, with a whole subtree behind it."""
        seed = self.ws.write(
            "sim",
            "launch/sim.launch.yaml",
            "launch:\n  - include:\n"
            '      file: "$(find-pkg-share $(var robot)_gazebo)/launch/spawn.launch.py"\n',
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertEqual(1, len(walk.unresolved))
        self.assertIn("$(var robot)_gazebo", walk.unresolved[0].package)
        self.assertEqual(3, walk.unresolved[0].line)

    def test_an_include_of_a_package_that_is_not_installed_is_reported(self):
        seed = self.ws.write(
            "app",
            "launch/app.launch.yaml",
            'launch:\n  - include:\n      file: "$(find-pkg-share absent)/launch/x.launch.yaml"\n',
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertEqual(["absent"], [e.package for e in walk.unresolved])

    def test_an_include_of_a_file_that_is_not_there_is_reported(self):
        """The package exists and the file does not - a rename, or a missing install rule."""
        self.ws.write("present", "launch/other.launch.yaml", "launch: []\n")
        seed = self.ws.write(
            "app",
            "launch/app.launch.yaml",
            'launch:\n  - include:\n      file: "$(find-pkg-share present)/launch/gone.launch.yaml"\n',
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertEqual(1, len(walk.unresolved))
        self.assertEqual("launch/gone.launch.yaml", walk.unresolved[0].launch_file)

    def test_a_data_file_is_not_mistaken_for_an_include(self):
        """`$(find-pkg-share x)/config/params.yaml` has an extension we scan but is not an
        include."""
        seed = self.ws.write(
            "app",
            "launch/app.launch.yaml",
            "launch:\n  - node:\n      pkg: app_node\n"
            '      param: "$(find-pkg-share app)/config/params.yaml"\n',
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertEqual([], walk.unresolved)


class TestProvenance(ScanFromTestCase):
    """Every reference carries the file and line it was written on."""

    def test_each_reference_carries_its_line(self):
        seed = self.ws.write(
            "app",
            "launch/app.launch.yaml",
            "launch:\n"
            "  - node:\n"
            "      pkg: first_package\n"
            "  - node:\n"
            "      pkg: second_package\n",
        )

        by_name = {r.package: r.line for r in scan_file_detailed(seed)}

        self.assertEqual(3, by_name["first_package"])
        self.assertEqual(5, by_name["second_package"])

    def test_a_multi_line_comment_does_not_shift_the_lines_after_it(self):
        """Comments are blanked out, not deleted, so later line numbers still hold."""
        seed = self.ws.write(
            "app",
            "launch/app.launch.xml",
            "<launch>\n"
            "  <!-- a comment\n"
            "       spanning\n"
            "       several lines -->\n"
            '  <node pkg="after_the_comment"/>\n'
            "</launch>\n",
        )

        found = scan_file_detailed(seed)

        self.assertEqual(1, len(found))
        self.assertEqual(5, found[0].line)


class TestIncludesWrittenInPython(ScanFromTestCase):
    """A tree can hide most of itself behind a single `IncludeLaunchDescription`."""

    def test_the_documented_shape_is_followed(self):
        self.ws.write(
            "described",
            "launch/description.launch.py",
            "def generate_launch_description():\n"
            "    return [Node(package='only_named_downstream', executable='x')]\n",
        )
        seed = self.ws.write(
            "sim",
            "launch/spawn.launch.py",
            "from launch.actions import IncludeLaunchDescription\n"
            "def generate_launch_description():\n"
            "    return [IncludeLaunchDescription(\n"
            "        PathJoinSubstitution([\n"
            "            FindPackageShare('described'), 'launch', 'description.launch.py',\n"
            "        ])\n"
            "    )]\n",
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertIn("only_named_downstream", walk.packages)
        self.assertEqual([], walk.unresolved)

    def test_the_os_path_join_shape_is_followed(self):
        self.ws.write(
            "other", "launch/child.launch.py", "Node(package='deep_package')\n"
        )
        seed = self.ws.write(
            "app",
            "launch/app.launch.py",
            "IncludeLaunchDescription(\n"
            "    os.path.join(get_package_share_directory('other'), 'launch', 'child.launch.py')\n"
            ")\n",
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertIn("deep_package", walk.packages)

    def test_a_lookup_hoisted_to_a_constant_is_followed(self):
        """`GZ = get_package_share_directory("gz")` at module level, used further down."""
        self.ws.write("gz", "launch/gz.launch.py", "Node(package='gz_internals')\n")
        seed = self.ws.write(
            "app",
            "launch/app.launch.py",
            "GZ = get_package_share_directory('gz')\n"
            "IncludeLaunchDescription(\n"
            "    PathJoinSubstitution([GZ, 'launch', 'gz.launch.py'])\n"
            ")\n",
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertIn("gz_internals", walk.packages)

    def test_a_qualified_call_is_followed(self):
        """`launch.actions.IncludeLaunchDescription(...)`, matched on the trailing name."""
        self.ws.write(
            "other", "launch/child.launch.py", "Node(package='deep_package')\n"
        )
        seed = self.ws.write(
            "app",
            "launch/app.launch.py",
            "import launch, ament_index_python\n"
            "launch.actions.IncludeLaunchDescription(\n"
            "    os.path.join(\n"
            "        ament_index_python.get_package_share_directory('other'),\n"
            "        'launch', 'child.launch.py')\n"
            ")\n",
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertIn("deep_package", walk.packages)
        self.assertEqual([], walk.unresolved)

    def test_a_qualified_call_it_cannot_read_is_still_reported(self):
        seed = self.ws.write(
            "app",
            "launch/app.launch.py",
            "import launch\n"
            "launch.actions.IncludeLaunchDescription(pick_a_launch_file())\n",
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertEqual(1, len(walk.unresolved))
        self.assertEqual(2, walk.unresolved[0].line)

    def test_an_include_it_cannot_read_is_reported_with_its_line(self):
        """An include assembled at runtime cannot be followed, so it is reported."""
        seed = self.ws.write(
            "app",
            "launch/app.launch.py",
            "def generate_launch_description():\n"
            "    chosen = pick_a_launch_file()\n"
            "    return [IncludeLaunchDescription(chosen)]\n",
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertEqual(1, len(walk.unresolved))
        self.assertEqual(3, walk.unresolved[0].line)
        self.assertIn("could not read", walk.unresolved[0].launch_file)


class TestSubstitutedArguments(ScanFromTestCase):
    """`$(var x)` inside a package name."""

    SEED = (
        "launch:\n"
        "  - arg:\n"
        "      name: robot\n"
        "      default: athena\n"
        "  - arg:\n"
        "      name: robot_name\n"
        '      default: "$(var robot)"\n'
        "  - include:\n"
        '      file: "$(find-pkg-share $(var robot_name)_sim)/launch/spawn.launch.yaml"\n'
    )

    def test_a_declared_default_resolves_the_include(self):
        """And through a chain: robot_name defaults to $(var robot), which defaults to athena."""
        self.ws.write(
            "athena_sim",
            "launch/spawn.launch.yaml",
            "launch:\n  - node:\n      pkg: found_only_behind_the_substitution\n",
        )
        seed = self.ws.write("scenario", "launch/robot.launch.yaml", self.SEED)

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertIn("found_only_behind_the_substitution", walk.packages)
        self.assertEqual([], walk.unresolved)

    def test_the_callers_value_wins_over_the_default(self):
        """A caller that knows its arguments follows the tree that will come up."""
        self.ws.write(
            "telemax_sim",
            "launch/spawn.launch.yaml",
            "launch:\n  - node:\n      pkg: a_telemax_only_package\n",
        )
        seed = self.ws.write("scenario", "launch/robot.launch.yaml", self.SEED)

        walk = scan_from(seed, share_lookup=self.ws.lookup, args={"robot": "telemax"})

        self.assertIn("a_telemax_only_package", walk.packages)

    def test_an_unknown_argument_leaves_the_include_unresolved(self):
        """Fails safe: what cannot be expanded is reported, never guessed."""
        seed = self.ws.write(
            "scenario",
            "launch/robot.launch.yaml",
            "launch:\n  - include:\n"
            '      file: "$(find-pkg-share $(var never_declared)_sim)/launch/x.launch.yaml"\n',
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertEqual(1, len(walk.unresolved))
        self.assertIn("$(var never_declared)", walk.unresolved[0].package)

    def test_arguments_defined_in_terms_of_each_other_do_not_hang(self):
        from package_xml_validation.helpers.find_launch_dependencies import expand_vars

        self.assertEqual(
            "$(var a)", expand_vars("$(var a)", {"a": "$(var b)", "b": "$(var a)"})
        )

    def test_a_launch_file_that_is_not_valid_yaml_expands_nothing(self):
        """An unparsable file is not a crash and not a guess - the names stay as written."""
        from package_xml_validation.helpers.find_launch_dependencies import (
            declared_args,
        )

        broken = self.ws.write(
            "app", "launch/broken.launch.yaml", "launch:\n  - [unclosed\n"
        )

        self.assertEqual({}, declared_args(broken))


class TestScanningAWholeDirectory(ScanFromTestCase):
    """`scan_files` plus recursion, as one walk over every seed rather than one walk each."""

    def test_every_launch_file_in_the_directory_is_a_seed(self):
        self.ws.write(
            "app", "launch/one.launch.yaml", "launch:\n  - node:\n      pkg: from_one\n"
        )
        self.ws.write(
            "app", "launch/two.launch.yaml", "launch:\n  - node:\n      pkg: from_two\n"
        )

        from package_xml_validation.helpers.find_launch_dependencies import (
            scan_directory,
        )

        walk = scan_directory(
            os.path.join(self.ws.root, "app"), share_lookup=self.ws.lookup
        )

        self.assertIn("from_one", walk.packages)
        self.assertIn("from_two", walk.packages)

    def test_a_file_two_seeds_both_reach_is_read_once(self):
        """Sibling launch files include the same things; the shared visited set reads them once."""
        from package_xml_validation.helpers.find_launch_dependencies import (
            scan_directory,
        )

        self.ws.write(
            "common",
            "launch/shared.launch.yaml",
            "launch:\n  - node:\n      pkg: shared_dep\n",
        )
        for name in ("one", "two", "three"):
            self.ws.write(
                "app",
                f"launch/{name}.launch.yaml",
                'launch:\n  - include:\n      file: "$(find-pkg-share common)/launch/shared.launch.yaml"\n',
            )

        walk = scan_directory(
            os.path.join(self.ws.root, "app"), share_lookup=self.ws.lookup
        )

        read_shared = [f for f in walk.visited if f.endswith("shared.launch.yaml")]
        self.assertEqual(
            1, len(read_shared), "the shared include was read more than once"
        )
        self.assertEqual(4, len(walk.visited))

    def test_the_same_file_reached_twice_is_not_reported_twice(self):
        from package_xml_validation.helpers.find_launch_dependencies import (
            scan_directory,
        )

        self.ws.write(
            "common",
            "launch/shared.launch.yaml",
            "launch:\n  - node:\n      pkg: shared_dep\n",
        )
        for name in ("one", "two"):
            self.ws.write(
                "app",
                f"launch/{name}.launch.yaml",
                'launch:\n  - include:\n      file: "$(find-pkg-share common)/launch/shared.launch.yaml"\n',
            )

        walk = scan_directory(
            os.path.join(self.ws.root, "app"), share_lookup=self.ws.lookup
        )

        self.assertEqual(1, [r.package for r in walk.references].count("shared_dep"))


class TestFindingALaunchFileInsideAPackage(ScanFromTestCase):
    """A component names its launch file by bare filename; the manager finds it wherever it is."""

    def test_a_launch_file_in_a_subdirectory_is_found(self):
        self.ws.write(
            "nav",
            "launch/navigation/nav2.launch.yaml",
            "launch:\n  - node:\n      pkg: a_planner\n",
        )
        seed = self.ws.write(
            "config",
            "components/nav2.yaml",
            "launch:\n  package: nav\n  launch_file: nav2.launch.yaml\n",
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertIn("a_planner", walk.packages)
        self.assertEqual([], walk.unresolved)

    def test_a_launch_file_that_exists_nowhere_stays_unresolved(self):
        """The fallback must not turn a real finding into a pass."""
        self.ws.write("driver", "launch/something_else.launch.yaml", "launch: []\n")
        seed = self.ws.write(
            "config",
            "components/broken.yaml",
            "launch:\n  package: driver\n  launch_file: not_anywhere.launch.yaml\n",
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertEqual(1, len(walk.unresolved))
        self.assertIn("not_anywhere.launch.yaml", walk.unresolved[0].launch_file)

    def test_a_written_out_path_does_not_fall_back_to_the_basename(self):
        """The basename search is for components only. Elsewhere a renamed or uninstalled
        launch file must not resolve to a same-named one elsewhere in the package."""
        self.ws.write(
            "present",
            "somewhere_else/gone.launch.yaml",
            "launch:\n  - node:\n      pkg: from_the_wrong_file\n",
        )
        seed = self.ws.write(
            "app",
            "launch/app.launch.yaml",
            'launch:\n  - include:\n      file: "$(find-pkg-share present)/launch/gone.launch.yaml"\n',
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertEqual(1, len(walk.unresolved))
        self.assertNotIn("from_the_wrong_file", walk.packages)


class TestComponentBlocks(ScanFromTestCase):
    """`package` and `launch_file` have to belong to the same mapping."""

    def test_a_node_only_component_does_not_borrow_the_next_launch_file(self):
        """A component with no `launch_file` of its own is not an include, however close the
        next component's is."""
        self.ws.write(
            "pkg_b",
            "launch/b.launch.yaml",
            "launch:\n  - node:\n      pkg: only_under_pkg_b\n",
        )
        seed = self.ws.write(
            "config",
            "components/two.yaml",
            "components:\n"
            "  - name: one\n"
            "    package: pkg_a\n"
            "    node: some_node\n"
            "  - name: two\n"
            "    package: pkg_b\n"
            "    launch_file: b.launch.yaml\n",
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertEqual([], walk.unresolved)
        self.assertIn("only_under_pkg_b", walk.packages)

    def test_an_unparsable_file_yields_no_component_edges(self):
        seed = self.ws.write(
            "config", "components/broken.yaml", "components:\n  - [unclosed\n"
        )

        walk = scan_from(seed, share_lookup=self.ws.lookup)

        self.assertEqual([], walk.unresolved)
