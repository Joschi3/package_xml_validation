import unittest

import lxml.etree as ET

from package_xml_validation.helpers.formatter.constants import (
    ELEMENTS,
    NEW_LINE_BEFORE,
)
from package_xml_validation.helpers.formatter.dependency_queries import (
    retrieve_all_dependencies,
)
from package_xml_validation.helpers.formatter.structural_checks import (
    check_for_non_existing_tags,
)
from package_xml_validation.helpers.logger import get_logger


class TestFormat3OptionalTags(unittest.TestCase):
    """REP-149 lists ``<conflict>`` and ``<replace>`` as valid top-level
    dependency tags. Confirm the validator accepts them."""

    def setUp(self):
        self.logger = get_logger("test_format3_optional_tags", level="normal")

    def test_conflict_and_replace_in_elements_schema(self):
        tags = [e[0] for e in ELEMENTS]
        self.assertIn("conflict", tags)
        self.assertIn("replace", tags)

    def test_conflict_and_replace_open_a_visual_block(self):
        self.assertIn("conflict", NEW_LINE_BEFORE)
        self.assertIn("replace", NEW_LINE_BEFORE)

    def test_unknown_tag_check_accepts_conflict_and_replace(self):
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <version>0.0.1</version>
  <description>demo</description>
  <maintainer email="a@example.com">Alex</maintainer>
  <license>Apache-2.0</license>
  <conflict>old_pkg</conflict>
  <replace>legacy_pkg</replace>
</package>
"""
        root = ET.fromstring(xml.encode("utf-8"))
        self.assertTrue(check_for_non_existing_tags(root, "inline.xml", self.logger))

    def test_conflict_and_replace_not_treated_as_installable_deps(self):
        """REP-149 anti-dependencies must not be sent to rosdep."""
        xml = """<?xml version="1.0"?>
<package format="3">
  <name>demo</name>
  <depend>real_dep</depend>
  <conflict>old_pkg</conflict>
  <replace>legacy_pkg</replace>
</package>
"""
        root = ET.fromstring(xml.encode("utf-8"))
        deps = retrieve_all_dependencies(root)
        self.assertIn("real_dep", deps)
        self.assertNotIn("old_pkg", deps)
        self.assertNotIn("legacy_pkg", deps)


if __name__ == "__main__":
    unittest.main()
