import importlib
import os
import sys
import tempfile
import unittest
from types import ModuleType
from pathlib import Path
from unittest.mock import patch

import yaml


def _install_fake_rosdep(
    mapping_values,
    *,
    raise_on_init=False,
    rule_for_platform=("apt", {}),
):
    fake_rosdep2 = ModuleType("rosdep2")
    fake_rospkg_loader = ModuleType("rosdep2.rospkg_loader")
    fake_rospkg_loader.DEFAULT_VIEW_KEY = "default"

    class DummyInstallerContext:
        def get_os_name_and_version(self):
            return ("ubuntu", "focal")

    class DummyDep:
        def get_rule_for_platform(self, os_name, os_version, installers, default):
            return rule_for_platform

    class DummyView:
        def __init__(self, keys):
            self._keys = set(keys)

        def keys(self):
            return self._keys

        def lookup(self, key):
            if key not in self._keys:
                raise KeyError(key)
            return DummyDep()

    class DummyLookup:
        def __init__(self, keys):
            self._keys = keys

        def get_rosdep_view(self, view_key):
            return DummyView(self._keys)

        @classmethod
        def create_from_rospkg(cls):
            return cls(mapping_values)

    if raise_on_init:

        def _raise():
            raise RuntimeError("rosdep init failure")

        fake_rosdep2.create_default_installer_context = _raise
    else:
        fake_rosdep2.create_default_installer_context = DummyInstallerContext
    fake_rosdep2.RosdepLookup = DummyLookup

    sys.modules["rosdep2"] = fake_rosdep2
    sys.modules["rosdep2.rospkg_loader"] = fake_rospkg_loader


def _reload_verify_module():
    module_name = "package_xml_validation.helpers.verify_rosdep_mapping"
    if module_name in sys.modules:
        del sys.modules[module_name]
    return importlib.import_module(module_name)


class TestVerifyRosdepMapping(unittest.TestCase):
    def setUp(self):
        self._original_cwd = os.getcwd()
        self._original_rosdep = sys.modules.get("rosdep2")
        self._original_rosdep_loader = sys.modules.get("rosdep2.rospkg_loader")

    def tearDown(self):
        os.chdir(self._original_cwd)
        if self._original_rosdep is not None:
            sys.modules["rosdep2"] = self._original_rosdep
        else:
            sys.modules.pop("rosdep2", None)
        if self._original_rosdep_loader is not None:
            sys.modules["rosdep2.rospkg_loader"] = self._original_rosdep_loader
        else:
            sys.modules.pop("rosdep2.rospkg_loader", None)

    def test_verify_success_with_valid_mapping(self):
        repo_root = Path(__file__).resolve().parents[1]
        os.chdir(repo_root)

        mapping_path = repo_root / "package_xml_validation/data/cmake_rosdep_map.yaml"
        mapping = yaml.safe_load(mapping_path.read_text()) or {}
        mapping_values = [value for value in mapping.values() if isinstance(value, str)]

        _install_fake_rosdep(mapping_values)
        module = _reload_verify_module()

        with self.assertRaises(SystemExit) as excinfo:
            module.verify_mappings()
        self.assertEqual(excinfo.exception.code, 0)

    def test_verify_fails_when_mapping_missing(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            os.chdir(tmp_dir)
            _install_fake_rosdep([])
            module = _reload_verify_module()

            with self.assertRaises(SystemExit) as excinfo:
                module.verify_mappings()
            self.assertEqual(excinfo.exception.code, 1)

    def test_verify_fails_on_missing_rosdep_key(self):
        repo_root = Path(__file__).resolve().parents[1]
        os.chdir(repo_root)

        _install_fake_rosdep([])
        module = _reload_verify_module()

        with patch.object(
            module.yaml, "safe_load", return_value={"BadKey": "missing_key"}
        ):
            with self.assertRaises(SystemExit) as excinfo:
                module.verify_mappings()
        self.assertEqual(excinfo.exception.code, 1)

    def test_verify_skips_non_string_values(self):
        repo_root = Path(__file__).resolve().parents[1]
        os.chdir(repo_root)

        _install_fake_rosdep([])
        module = _reload_verify_module()

        with patch.object(module.yaml, "safe_load", return_value={"Key": ["list"]}):
            with self.assertRaises(SystemExit) as excinfo:
                module.verify_mappings()
        self.assertEqual(excinfo.exception.code, 0)

    def test_verify_reports_rosdep_init_failure(self):
        repo_root = Path(__file__).resolve().parents[1]
        os.chdir(repo_root)

        _install_fake_rosdep([], raise_on_init=True)
        module = _reload_verify_module()

        with self.assertRaises(SystemExit) as excinfo:
            module.verify_mappings()
        self.assertEqual(excinfo.exception.code, 1)


if __name__ == "__main__":
    unittest.main()
