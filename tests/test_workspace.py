# tests/test_workspace.py
import io
import sys
import unittest
import tempfile
from pathlib import Path
from contextlib import redirect_stdout
from unittest import mock

from package_xml_validation.helpers import workspace as SUT


class TempTree:
    def __init__(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)

    def ws(self, name="ws"):
        ws = self.root / name
        (ws / "src").mkdir(parents=True, exist_ok=True)
        return ws

    def pkg(self, base: Path, pkg_name: str, *, name_in_xml=None, malformed=False):
        d = base / pkg_name
        d.mkdir(parents=True, exist_ok=True)
        xml = d / "package.xml"
        if malformed:
            xml.write_text("<package><name>oops")  # invalid XML
        else:
            if name_in_xml is None:
                xml.write_text("<package></package>")
            else:
                xml.write_text(f"<package><name>{name_in_xml}</name></package>")
        return d

    def touch(self, p: Path, content: str = ""):
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(content)
        return p

    def close(self):
        self._tmp.cleanup()


class TestWorkspaceHelpers(unittest.TestCase):
    def setUp(self):
        self.t = TempTree()

        # Workspace with packages
        self.ws = self.t.ws("ws_ok")
        src = self.ws / "src"
        self.pkg1 = self.t.pkg(src, "pkg1", name_in_xml="pkg1")
        self.pkg_nameless = self.t.pkg(src, "pkg_nameless", name_in_xml=None)

        self.nested_parent = src / "nested"
        self.inner_pkg = self.t.pkg(
            self.nested_parent, "inner_pkg", name_in_xml="inner_pkg"
        )

        # Ignored via COLCON_IGNORE in package dir
        self.pkg_ignored = self.t.pkg(src, "pkg_ignored", name_in_xml="pkg_ignored")
        (self.pkg_ignored / "COLCON_IGNORE").write_text("")

        # Ignored via parent directory COLCON_IGNORE
        third_party = src / "third_party"
        third_party.mkdir(parents=True, exist_ok=True)  # <-- ensure parent exists
        (third_party / "COLCON_IGNORE").write_text("")
        self.pkg_third = self.t.pkg(third_party, "third_pkg", name_in_xml="third_pkg")

        # File inside pkg1
        self.t.touch(self.pkg1 / "node.py", "#!/usr/bin/env python3")

        # Non-workspace flat folder; used to test fallback in get_pkgs_in_wrs
        self.flat = self.t.root / "flat_pkgs"
        self.flat.mkdir(parents=True, exist_ok=True)
        self.flat_a = self.t.pkg(self.flat, "flat_a", name_in_xml="flat_a")
        self.flat_b = self.t.pkg(self.flat, "flat_b", name_in_xml="flat_b")
        self.t.touch(self.flat_a / "script.py", "#!/usr/bin/env python3")

    def tearDown(self):
        self.t.close()

    # --- parse_pkg_name -----------------------------------------------------
    def test_parse_pkg_name_valid(self):
        self.assertEqual(SUT.parse_pkg_name(self.pkg1 / "package.xml"), "pkg1")

    def test_parse_pkg_name_missing_name_fallback(self):
        self.assertEqual(
            SUT.parse_pkg_name(self.pkg_nameless / "package.xml"), "pkg_nameless"
        )

    def test_parse_pkg_name_malformed_xml_fallback(self):
        badroot = self.t.root / "bad"
        badpkg = self.t.pkg(badroot, "badpkg", malformed=True)
        self.assertEqual(SUT.parse_pkg_name(badpkg / "package.xml"), "badpkg")

    # --- find_package_dir ---------------------------------------------------
    def test_find_package_dir_from_pkg_dir(self):
        self.assertEqual(SUT.find_package_dir(self.pkg1), self.pkg1)

    def test_find_package_dir_from_file_path(self):
        self.assertEqual(SUT.find_package_dir(self.pkg1 / "node.py"), self.pkg1)

    def test_find_package_dir_downward_search_ignores_build_install(self):
        ws2 = self.t.ws("ws2")
        _ = self.t.pkg(
            ws2 / "build", "fake_pkg", name_in_xml="fake_pkg"
        )  # should be ignored
        with self.assertRaises(ValueError):
            SUT.find_package_dir(ws2)

    def test_find_package_dir_no_package_raises(self):
        nowhere = self.t.root / "nowhere"
        nowhere.mkdir(parents=True, exist_ok=True)
        with self.assertRaises(ValueError):
            SUT.find_package_dir(nowhere)

    # --- looks_like_ws_root -------------------------------------------------
    def test_looks_like_ws_root_true(self):
        self.assertTrue(SUT.looks_like_ws_root(self.ws, self.pkg1))

    def test_looks_like_ws_root_false(self):
        not_ws = self.t.root / "not_ws"
        not_ws.mkdir(parents=True, exist_ok=True)
        self.assertFalse(SUT.looks_like_ws_root(not_ws, self.pkg1))

    # --- find_workspace_root ------------------------------------------------
    def test_find_workspace_root_from_pkg_dir(self):
        self.assertEqual(SUT.find_workspace_root(self.pkg1), self.ws)

    def test_find_workspace_root_from_file(self):
        self.assertEqual(SUT.find_workspace_root(self.pkg1 / "node.py"), self.ws)

    def test_find_workspace_root_raises_outside_ws(self):
        with self.assertRaises(ValueError):
            SUT.find_workspace_root(self.flat_a)

    # --- pkg_iterator -------------------------------------------------------
    def test_pkg_iterator_lists_pkgs_and_respects_colcon_ignore(self):
        pkgs = SUT.pkg_iterator(self.ws / "src")
        self.assertEqual(set(pkgs.keys()), {"inner_pkg", "pkg1", "pkg_nameless"})
        self.assertNotIn("pkg_ignored", pkgs)
        self.assertNotIn("third_pkg", pkgs)

    # --- get_pkgs_in_wrs ----------------------------------------------------
    def test_get_pkgs_in_wrs_with_string_path(self):
        names = SUT.get_pkgs_in_wrs(str(self.pkg1 / "node.py"))
        self.assertEqual(names, ["inner_pkg", "pkg1", "pkg_nameless"])

    def test_get_pkgs_in_wrs_with_path_obj(self):
        names = SUT.get_pkgs_in_wrs(self.pkg1 / "node.py")
        self.assertEqual(names, ["inner_pkg", "pkg1", "pkg_nameless"])

    def test_get_pkgs_in_wrs_fallback_flat_folder(self):
        names = SUT.get_pkgs_in_wrs(self.flat_a / "script.py")
        self.assertEqual(names, ["flat_a", "flat_b"])

    def test_get_pkgs_in_wrs_nonexistent_path_obj_raises_valueerror(self):
        bogus = self.t.root / "does" / "not" / "exist"
        with self.assertRaises(ValueError):
            SUT.get_pkgs_in_wrs(bogus)

    def test_get_pkgs_in_wrs_nonexistent_string_raises_filenotfound(self):
        bogus = str(self.t.root / "also" / "not" / "here")
        with self.assertRaises(FileNotFoundError):
            SUT.get_pkgs_in_wrs(bogus)

    # --- CLI main() ---------------------------------------------------------
    def test_main_lists_packages_names_only(self):
        argv = ["prog", str(self.pkg1)]
        with mock.patch.object(sys, "argv", argv):
            buf = io.StringIO()
            with redirect_stdout(buf):
                SUT.main()
            out = buf.getvalue().strip().splitlines()
        self.assertTrue(out[0].startswith("Workspace: "))
        listed = set(out[1:])
        self.assertTrue({"pkg1", "pkg_nameless", "inner_pkg"}.issubset(listed))
        self.assertNotIn("pkg_ignored", listed)
        self.assertFalse(any(s.startswith("third_pkg") for s in listed))

    def test_main_lists_packages_with_full_paths(self):
        argv = ["prog", "--full-paths", str(self.pkg1)]
        with mock.patch.object(sys, "argv", argv):
            buf = io.StringIO()
            with redirect_stdout(buf):
                SUT.main()
            lines = buf.getvalue().strip().splitlines()[1:]
        self.assertTrue(
            any(line.startswith("pkg1 ") and str(self.pkg1) in line for line in lines)
        )
        self.assertTrue(
            any(
                line.startswith("inner_pkg ") and str(self.inner_pkg) in line
                for line in lines
            )
        )
        self.assertTrue(
            any(
                line.startswith("pkg_nameless ") and str(self.pkg_nameless) in line
                for line in lines
            )
        )

    def test_main_exits_when_no_packages_found(self):
        ws2 = self.t.ws("ws2_empty")
        argv = ["prog", str(ws2)]
        with mock.patch.object(sys, "argv", argv), self.assertRaises(SystemExit):
            buf = io.StringIO()
            with redirect_stdout(buf):
                SUT.main()


if __name__ == "__main__":
    unittest.main()
