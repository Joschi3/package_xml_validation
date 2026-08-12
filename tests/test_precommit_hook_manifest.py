"""The contract `.pre-commit-hooks.yaml` makes with pre-commit.

Nothing else covers this file, and what it declares is not cosmetic: pre-commit throws away a
passing hook's output, so a hook that warns without `verbose: true` warns into nothing.
"""

import os
import re
import unittest

import yaml

MANIFEST = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), ".pre-commit-hooks.yaml"
)


def hook(hook_id):
    with open(MANIFEST, encoding="utf-8") as f:
        for entry in yaml.safe_load(f):
            if entry["id"] == hook_id:
                return entry
    raise AssertionError(f"no hook with id {hook_id!r} in {MANIFEST}")


class TestCheckLaunchTreeHook(unittest.TestCase):
    def setUp(self):
        self.hook = hook("check-launch-tree")

    def test_its_warnings_are_shown(self):
        """pre-commit prints a hook's output only when it fails, is verbose, or changed a
        file. This one exits zero on everything a CI runner cannot help, so without `verbose`
        those findings reach nobody."""
        self.assertIs(True, self.hook.get("verbose"))

    def test_it_is_not_handed_filenames(self):
        """It walks whole packages, not the files that happen to be staged."""
        self.assertIs(False, self.hook["pass_filenames"])

    def test_every_format_the_scanner_seeds_from_triggers_it(self):
        pattern = re.compile(self.hook["files"])
        for path in (
            "launch/a.launch.py",
            "launch/a.launch.xml",
            "launch/a.launch.yaml",
            "launch/a.launch.yml",
            "launch/a.launch",
            "launch/a.toml",
            "components/a.yaml",
            "src/pkg/launch_manager_components/a.yaml",
            "src/pkg/launch_manager_configs/a.yaml",
        ):
            self.assertRegex(path, pattern)

    def test_a_directory_merely_ending_in_launch_does_not(self):
        pattern = re.compile(self.hook["files"])
        for path in ("mylaunch/a.py", "src/prelaunch/a.yaml", "docs/launching.md"):
            self.assertNotRegex(path, pattern)


class TestPreCommitAcceptsTheManifest(unittest.TestCase):
    """Schema validation, when pre-commit itself is around to do it."""

    def test_it_loads(self):
        try:
            from pre_commit.clientlib import load_manifest
        except ImportError:
            self.skipTest("pre-commit is not installed")

        ids = [entry["id"] for entry in load_manifest(MANIFEST)]

        self.assertIn("check-launch-tree", ids)
        self.assertIn("format-package-xml", ids)
