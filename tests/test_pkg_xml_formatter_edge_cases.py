import tempfile
import unittest
from pathlib import Path

import lxml.etree as ET

from package_xml_validation.helpers.pkg_xml_formatter import PackageXmlFormatter


class TestPackageXmlFormatterEdgeCases(unittest.TestCase):
    def test_duplicate_detection_respects_attributes(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <maintainer email="a@example.com">Alex</maintainer>
  <maintainer email="b@example.com">Alex</maintainer>
</package>
"""
        root = ET.fromstring(xml.encode("utf-8"))
        formatter = PackageXmlFormatter(check_only=True)
        self.assertTrue(formatter.check_for_duplicates(root, "inline.xml"))

    def test_minimal_package_xml_auto_fix_does_not_crash(self):
        xml = """<?xml version="1.0"?><package format="3"><name>demo</name></package>"""
        root = ET.fromstring(xml.encode("utf-8"))
        formatter = PackageXmlFormatter(check_only=False)

        with tempfile.TemporaryDirectory() as tmp_dir:
            xml_path = Path(tmp_dir) / "package.xml"
            xml_path.write_text(xml, encoding="utf-8")
            result = formatter.check_for_empty_lines(root, str(xml_path))

        self.assertFalse(result)


if __name__ == "__main__":
    unittest.main()
