import logging
import tempfile
import unittest
from pathlib import Path

import lxml.etree as ET

from package_xml_validation.helpers.formatter.structural_checks import (
    check_for_empty_lines,
    check_indentation,
)
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


class TestCheckIndentationCommentNodes(unittest.TestCase):
    """Regression: comment nodes must not leak as `<cyfunction Comment>` in
    the "newlines in text" error message. Multi-line comments are legal —
    the rule only applies to schema elements like <description>."""

    def setUp(self):
        self.logger = logging.getLogger("test_indent_comments")

    def test_multiline_comment_does_not_trigger_newline_error(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <!-- this is a multi-line
       comment that legitimately
       spans several lines -->
</package>
"""
        root = ET.fromstring(xml.encode("utf-8"))
        with self.assertNoLogs(self.logger, level="ERROR"):
            check_indentation(root, check_only=True, logger=self.logger)

    def test_multiline_comment_with_xml_inside_does_not_leak_cyfunction(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <!-- <exec_depend>old_pkg</exec_depend>
       <exec_depend>another_pkg</exec_depend> -->
</package>
"""
        root = ET.fromstring(xml.encode("utf-8"))
        # Capture every error-level message; assert none mentions "cyfunction".
        with self.assertLogs(self.logger, level="DEBUG") as captured:
            self.logger.debug(
                "anchor"
            )  # ensure the context manager has at least one record
            check_indentation(root, check_only=True, logger=self.logger)
        for record in captured.output:
            self.assertNotIn("cyfunction", record)
            self.assertNotIn("Comment at 0x", record)


class TestSiblingSeparationMessages(unittest.TestCase):
    """Regression: ``check_for_empty_lines`` enforces a single rule —
    siblings must be separated by exactly one newline, with at most one
    blank line. Violations now report ``Sibling separation error`` rather
    than the older mismatched ``Two elements on the same line`` /
    ``More than one empty line found`` messages."""

    def setUp(self):
        self.logger = logging.getLogger("test_sibling_sep")

    def test_two_siblings_on_same_line_uses_unified_label(self):
        # No newline in a tail → two siblings on the same line.
        xml = (
            '<?xml version="1.0"?>\n'
            '<package format="3"><name>demo</name><version>0.0.1</version></package>'
        )
        root = ET.fromstring(xml.encode("utf-8"))
        with self.assertLogs(self.logger, level="INFO") as captured:
            check_for_empty_lines(
                root, "inline.xml", check_only=True, logger=self.logger
            )
        joined = "\n".join(captured.output)
        self.assertIn("Sibling separation error", joined)
        self.assertIn("two siblings on the same line", joined)

    def test_more_than_one_blank_line_uses_unified_label(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>


  <version>0.0.1</version>
</package>
"""
        root = ET.fromstring(xml.encode("utf-8"))
        with self.assertLogs(self.logger, level="INFO") as captured:
            check_for_empty_lines(
                root, "inline.xml", check_only=True, logger=self.logger
            )
        joined = "\n".join(captured.output)
        self.assertIn("Sibling separation error", joined)
        self.assertIn("more than one blank line", joined)


class TestResolveIndentation(unittest.TestCase):
    """``resolve_indentation`` exposes three fallback branches that
    callers rely on when a tree is malformed or freshly constructed."""

    def test_returns_default_when_root_is_empty(self):
        from package_xml_validation.helpers.formatter.indentation import (
            resolve_indentation,
        )

        root = ET.fromstring(b'<package format="3"/>')
        self.assertEqual(resolve_indentation(root), "  ")
        self.assertEqual(resolve_indentation(root, default="\t"), "\t")

    def test_returns_default_when_first_child_tail_is_none(self):
        from package_xml_validation.helpers.formatter.indentation import (
            resolve_indentation,
        )

        # Build a tree where the first child has tail=None explicitly.
        root = ET.Element("package")
        child = ET.SubElement(root, "name")
        child.text = "demo"
        # lxml's default tail for a SubElement is None.
        self.assertIsNone(child.tail)
        self.assertEqual(resolve_indentation(root), "  ")

    def test_returns_full_tail_when_no_newline(self):
        from package_xml_validation.helpers.formatter.indentation import (
            resolve_indentation,
        )

        root = ET.Element("package")
        child = ET.SubElement(root, "name")
        child.text = "demo"
        child.tail = "   "  # whitespace-only, no newline
        self.assertEqual(resolve_indentation(root), "   ")


if __name__ == "__main__":
    unittest.main()
