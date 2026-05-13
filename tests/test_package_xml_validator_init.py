"""Tests for ``PackageXmlValidator`` constructor flag interactions and
its handling of malformed input files. These paths are not exercised by
the higher-level integration tests in ``test_cmake_comparison.py``
because that suite always passes well-formed package.xml inputs and
constructs the validator with both ``check_rosdeps=True`` and
``compare_with_cmake=True``."""

import os
import tempfile
import unittest

from package_xml_validation.package_xml_validator import PackageXmlValidator

_VALIDATOR_LOGGER_NAME = "package_xml_validation.package_xml_validator"


class TestPackageXmlValidatorInit(unittest.TestCase):
    """Item 9: the constructor must silently disable ``compare_with_cmake``
    when ``check_rosdeps`` is False, emitting one warning explaining why."""

    def test_compare_with_cmake_without_check_rosdeps_is_disabled_with_warning(self):
        with self.assertLogs(_VALIDATOR_LOGGER_NAME, level="WARNING") as captured:
            validator = PackageXmlValidator(
                check_only=True,
                check_rosdeps=False,
                compare_with_cmake=True,
            )
        self.assertFalse(validator.compare_with_cmake)
        self.assertTrue(
            any(
                "Comparing with CMake but not checking ROS dependencies" in record
                for record in captured.output
            ),
            f"Expected warning not found in {captured.output!r}",
        )


class TestPackageXmlValidatorMalformedXml(unittest.TestCase):
    """Item 10: a malformed package.xml must not crash the run; the
    validator records ``xml_valid=False`` for that file, sets
    ``all_valid=False``, logs the parse error, and continues."""

    def test_malformed_package_xml_marks_invalid_and_continues(self):
        with tempfile.TemporaryDirectory(prefix="malformed_pkg_xml_") as tmp:
            pkg_dir = os.path.join(tmp, "demo")
            os.makedirs(pkg_dir)
            xml_path = os.path.join(pkg_dir, "package.xml")
            with open(xml_path, "w", encoding="utf-8") as f:
                # Deliberately unclosed element — ET.parse must raise
                # ET.XMLSyntaxError, which the validator catches.
                f.write("<package><name>oops")

            validator = PackageXmlValidator(
                check_only=True,
                check_rosdeps=False,
                compare_with_cmake=False,
            )

            with self.assertLogs(_VALIDATOR_LOGGER_NAME, level="ERROR") as captured:
                all_valid = validator.check_and_format_files([xml_path])

            self.assertFalse(all_valid)
            self.assertFalse(validator.xml_valid)
            self.assertFalse(validator.all_valid)
            self.assertTrue(
                any(
                    f"Error parsing {xml_path}" in record for record in captured.output
                ),
                f"Expected parse-error log not found in {captured.output!r}",
            )


if __name__ == "__main__":
    unittest.main()
