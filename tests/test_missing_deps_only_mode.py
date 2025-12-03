import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

from package_xml_validation.package_xml_validator import PackageXmlValidator


def _write_package_xml(path: Path, extra_body: str = ""):
    content = f"""<?xml version="1.0"?>
<package format="3">
  <name>demo_pkg</name>
  <version>0.0.0</version>
  <description>Demo package</description>
  <license>MIT</license>
  {extra_body}
</package>
"""
    path.write_text(content, encoding="utf-8")


class TestMissingDepsOnlyMode(unittest.TestCase):
    def setUp(self):
        self.tmpdir = Path(tempfile.mkdtemp(prefix="missing_deps_only_"))
        self.pkg_dir = self.tmpdir / "demo_pkg"
        self.pkg_dir.mkdir()
        self.package_xml = self.pkg_dir / "package.xml"

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_skips_formatting_checks(self):
        """missing_deps_only should bypass non-dependency checks."""
        _write_package_xml(self.package_xml)

        validator = PackageXmlValidator(
            missing_deps_only=True, check_rosdeps=False, verbose=True
        )
        self.assertTrue(validator.check_only)
        validator.formatter.check_for_non_existing_tags = MagicMock(return_value=False)

        with open(self.package_xml, "rb") as f:
            before = f.read()

        result = validator.check_and_format_files([str(self.package_xml)])

        self.assertTrue(result)
        validator.formatter.check_for_non_existing_tags.assert_not_called()
        with open(self.package_xml, "rb") as f:
            after = f.read()
        self.assertEqual(before, after)

    def test_detects_missing_launch_dependency(self):
        """missing_deps_only should still flag missing launch dependencies."""
        _write_package_xml(self.package_xml, "<exec_depend>rclpy</exec_depend>")
        launch_dir = self.pkg_dir / "launch"
        launch_dir.mkdir()
        (launch_dir / "demo.launch.py").write_text(
            "from launch_ros.actions import Node\nnode = Node(package='missing_pkg', executable='demo')\n",
            encoding="utf-8",
        )

        validator = PackageXmlValidator(
            missing_deps_only=True, check_rosdeps=False, verbose=True
        )

        with open(self.package_xml, "rb") as f:
            before = f.read()

        result = validator.check_and_format_files([str(self.package_xml)])

        self.assertFalse(result)
        with open(self.package_xml, "rb") as f:
            after = f.read()
        self.assertEqual(before, after)


if __name__ == "__main__":
    unittest.main()
