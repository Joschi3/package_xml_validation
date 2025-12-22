import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from package_xml_validation.package_xml_validator import PackageXmlValidator
from package_xml_validation.helpers.validation_steps import BuildToolDependStep


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


class TestIgnoreFormattingErrorsMode(unittest.TestCase):
    def setUp(self):
        self.tmpdir = Path(tempfile.mkdtemp(prefix="ignore_formatting_"))
        self.pkg_dir = self.tmpdir / "demo_pkg"
        self.pkg_dir.mkdir()
        self.package_xml = self.pkg_dir / "package.xml"

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_skips_formatting_checks(self):
        extra = """
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
"""
        _write_package_xml(self.package_xml, extra)

        validator = PackageXmlValidator(
            ignore_formatting_errors=True, check_rosdeps=False, verbose=True
        )
        self.assertTrue(validator.check_only)

        validator.formatter.check_for_empty_lines = MagicMock(return_value=False)
        validator.formatter.check_indentation = MagicMock(return_value=False)

        with open(self.package_xml, "rb") as f:
            before = f.read()

        with patch.object(
            BuildToolDependStep,
            "perform_check",
            autospec=True,
            wraps=BuildToolDependStep.perform_check,
        ) as buildtool_check:
            result = validator.check_and_format_files([str(self.package_xml)])

        self.assertTrue(result)
        validator.formatter.check_for_empty_lines.assert_not_called()
        validator.formatter.check_indentation.assert_not_called()
        buildtool_check.assert_called_once()

        with open(self.package_xml, "rb") as f:
            after = f.read()
        self.assertEqual(before, after)

    def test_still_reports_structural_errors(self):
        _write_package_xml(self.package_xml)

        validator = PackageXmlValidator(
            ignore_formatting_errors=True, check_rosdeps=False, verbose=True
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
