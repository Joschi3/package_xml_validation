import os
import tempfile
import unittest

import lxml.etree as ET

from package_xml_validation.helpers.validation_steps import (
    ManifestSchemaStep,
    ValidationConfig,
)


def _config(**overrides):
    base = {
        "check_only": True,
        "auto_fill_missing_deps": False,
        "check_rosdeps": False,
        "compare_with_cmake": False,
        "strict_cmake_checking": False,
        "missing_deps_only": False,
        "ignore_formatting_errors": False,
    }
    base.update(overrides)
    return ValidationConfig(**base)


def _parse(xml: str):
    return ET.fromstring(xml.encode("utf-8"))


VALID = """<?xml version="1.0"?>
<package format="3">
  <name>demo_pkg</name>
  <version>1.2.3</version>
  <description>demo</description>
  <maintainer email="a@example.com">Alex</maintainer>
  <license>Apache-2.0</license>
</package>
"""


class TestManifestSchemaStep(unittest.TestCase):
    def setUp(self):
        self.step = ManifestSchemaStep(_config())
        tmp = tempfile.TemporaryDirectory(prefix="manifest_schema_")
        self.addCleanup(tmp.cleanup)
        self.fake_xml = os.path.join(tmp.name, "demo_pkg", "package.xml")

    def test_valid_manifest_passes(self):
        result = self.step.perform_check(_parse(VALID), self.fake_xml)
        self.assertTrue(result.valid)
        self.assertEqual(result.errors, [])
        self.assertEqual(result.warnings, [])

    def test_format_2_warns_but_passes(self):
        xml = VALID.replace('format="3"', 'format="2"')
        result = self.step.perform_check(_parse(xml), self.fake_xml)
        self.assertTrue(result.valid)
        self.assertEqual(len(result.warnings), 1)
        self.assertIn('format="2"', result.warnings[0])

    def test_missing_format_errors(self):
        xml = VALID.replace('format="3"', "")
        result = self.step.perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)
        self.assertTrue(any("Missing format" in e for e in result.errors))

    def test_unsupported_format_errors(self):
        xml = VALID.replace('format="3"', 'format="9"')
        result = self.step.perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)
        self.assertTrue(any('format="9"' in e for e in result.errors))

    def test_invalid_name_uppercase(self):
        xml = VALID.replace("<name>demo_pkg</name>", "<name>DemoPkg</name>")
        result = self.step.perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)
        self.assertTrue(any("Invalid <name>DemoPkg" in e for e in result.errors))

    def test_invalid_name_hyphen(self):
        xml = VALID.replace("<name>demo_pkg</name>", "<name>demo-pkg</name>")
        result = self.step.perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)

    def test_invalid_name_starts_with_digit(self):
        xml = VALID.replace("<name>demo_pkg</name>", "<name>1demo</name>")
        result = self.step.perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)

    def test_invalid_version_non_numeric(self):
        xml = VALID.replace("<version>1.2.3</version>", "<version>1.2.3a</version>")
        result = self.step.perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)
        self.assertTrue(any("Invalid <version>" in e for e in result.errors))

    def test_invalid_version_four_parts(self):
        xml = VALID.replace("<version>1.2.3</version>", "<version>1.2.3.4</version>")
        result = self.step.perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)

    def test_maintainer_missing_email(self):
        xml = VALID.replace(
            '<maintainer email="a@example.com">Alex</maintainer>',
            "<maintainer>Alex</maintainer>",
        )
        result = self.step.perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)
        self.assertTrue(any("Alex" in e and "email" in e for e in result.errors))

    def test_maintainer_empty_email(self):
        xml = VALID.replace(
            '<maintainer email="a@example.com">Alex</maintainer>',
            '<maintainer email="">Alex</maintainer>',
        )
        result = self.step.perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)

    def test_multiple_maintainers_only_one_missing_email(self):
        xml = VALID.replace(
            '<maintainer email="a@example.com">Alex</maintainer>',
            '<maintainer email="a@example.com">Alex</maintainer>\n  <maintainer>Bob</maintainer>',
        )
        result = self.step.perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)
        self.assertEqual(len(result.errors), 1)
        self.assertIn("Bob", result.errors[0])

    def test_multiple_violations_each_reported(self):
        xml = """<?xml version="1.0"?>
<package>
  <name>BAD-NAME</name>
  <version>1.2</version>
  <maintainer>NoEmail</maintainer>
</package>
"""
        result = self.step.perform_check(_parse(xml), self.fake_xml)
        self.assertFalse(result.valid)
        self.assertGreaterEqual(len(result.errors), 4)

    def test_skipped_under_missing_deps_only(self):
        broken = VALID.replace("<name>demo_pkg</name>", "<name>BAD-NAME</name>")
        step = ManifestSchemaStep(_config(missing_deps_only=True))
        result = step.perform_check(_parse(broken), self.fake_xml)
        self.assertTrue(result.valid)
        self.assertEqual(result.errors, [])

    def test_skipped_under_ignore_formatting_errors(self):
        broken = VALID.replace("<name>demo_pkg</name>", "<name>BAD-NAME</name>")
        step = ManifestSchemaStep(_config(ignore_formatting_errors=True))
        result = step.perform_check(_parse(broken), self.fake_xml)
        self.assertTrue(result.valid)


if __name__ == "__main__":
    unittest.main()
