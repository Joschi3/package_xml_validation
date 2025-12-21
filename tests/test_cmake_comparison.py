import os
import unittest
import tempfile
import shutil
import subprocess
import lxml.etree as ET
from package_xml_validation.package_xml_validator import (
    PackageXmlValidator,
)
from unittest.mock import MagicMock

from package_xml_validation.helpers.pkg_xml_formatter import PackageXmlFormatter
from package_xml_validation.helpers.cmake_parsers import read_deps_from_cmake_file


def validate_xml_with_xmllint(xml_file):
    """Validate XML file against the ROS package_format3.xsd schema using xmllint."""
    schema_url = "http://download.ros.org/schema/package_format3.xsd"
    try:
        result = subprocess.run(
            ["xmllint", "--noout", "--schema", schema_url, xml_file],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            print(f"XML validation error in {xml_file}:\n{result.stderr}")
            return False
        return True
    except FileNotFoundError:
        print(
            "Error: xmllint not found. Please ensure it's installed and in your PATH."
        )
        return False


class TestPackageXmlValidator(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """
        We assume the example files are in 'tests/examples'.
        Adjust the directory path to match your actual setup.
        """
        current_dir = os.path.dirname(__file__)
        cls.examples_dir = os.path.join(current_dir, "examples", "cmake_specific_tests")

        # --- SETUP 1: Full Formatter ---
        cls.formatter = PackageXmlValidator(
            check_only=False,
            verbose=True,
            auto_fill_missing_deps=True,
            check_rosdeps=True,
            compare_with_cmake=True,
        )
        # Mock the rosdep validator
        cls.formatter.rosdep_validator = MagicMock()
        # 1. Mock the validation check to return "No errors"
        cls.formatter.rosdep_validator.check_rosdeps_and_local_pkgs.return_value = []
        # 2. CRITICAL FIX: Mock the resolution to return the input string (pass-through)
        # If we don't do this, it returns a MagicMock object, causing TypeError during string join
        cls.formatter.rosdep_validator.resolve_cmake_dependency.side_effect = (
            lambda x: x
        )

        # --- SETUP 2: Check-Only Formatter ---
        cls.formatter_check_only = PackageXmlValidator(
            check_only=True,
            verbose=True,
            auto_fill_missing_deps=True,
            check_rosdeps=True,
            compare_with_cmake=True,
        )
        # Mock the rosdep validator
        cls.formatter_check_only.rosdep_validator = MagicMock()
        cls.formatter_check_only.rosdep_validator.check_rosdeps_and_local_pkgs.return_value = []
        # CRITICAL FIX applied here as well
        cls.formatter_check_only.rosdep_validator.resolve_cmake_dependency.side_effect = (
            lambda x: x
        )

    def setUp(self):
        """
        Create a temporary directory for each test, copy example files into it,
        so we can safely modify them during tests.
        """
        self.test_dir = tempfile.mkdtemp(prefix="xml_tests_")
        shutil.copytree(self.examples_dir, self.test_dir, dirs_exist_ok=True)

    def tearDown(self):
        """Clean up the temporary directory after each test."""
        shutil.rmtree(self.test_dir)

    def _compare_xml_files(self, file1: str, file2: str) -> bool:
        """Compare two XML files for equality, ignoring whitespace/comments."""
        try:
            tree1 = ET.parse(file1)
            tree2 = ET.parse(file2)
            root1 = tree1.getroot()
            root2 = tree2.getroot()

            # Simple comparison of tags and text
            for elem1, elem2 in zip(root1.iter(), root2.iter()):
                if elem1.tag != elem2.tag:
                    return False
                # Strip text to ignore indentation changes
                text1 = (elem1.text or "").strip()
                text2 = (elem2.text or "").strip()
                if text1 != text2:
                    return False

            return True
        except ET.XMLSyntaxError as e:
            print(f"XML Syntax Error: {e}")
            return False

    def test_xml_formatting(self):
        """
        Iterate over all example packages in the test directory,
        """
        example_pkgs = os.listdir(self.examples_dir)
        for example_pkg in example_pkgs:
            correct_xml = os.path.join(
                self.examples_dir, example_pkg, "pkg_correct", "package.xml"
            )
            # Ensure correct_xml exists before proceeding
            if not os.path.exists(correct_xml):
                continue

            build_type_dir = os.path.join(self.test_dir, example_pkg)
            if not os.path.isdir(build_type_dir):
                continue

            for pkg in os.listdir(build_type_dir):
                xml_file = os.path.join(build_type_dir, pkg, "package.xml")
                if not os.path.exists(xml_file):
                    continue

                with self.subTest(example_pkg=example_pkg, pkg=pkg, xml_file=xml_file):
                    with open(xml_file) as f:
                        xml_content = f.read()

                    # 1. Test Check Only
                    valid = self.formatter_check_only.check_and_format_files([xml_file])

                    if pkg == "pkg_correct":
                        self.assertTrue(valid, f"Expected {xml_file} to be valid.")
                    else:
                        self.assertFalse(valid, f"Expected {xml_file} to be invalid.")
                        # Verify file was NOT modified
                        with open(xml_file) as f:
                            self.assertEqual(f.read(), xml_content)

                    # 2. Test Full Formatter (Correction)
                    valid = self.formatter.check_and_format_files([xml_file])

                    # After formatting, it returns False if it HAD to fix errors,
                    # but the file on disk should now match pkg_correct
                    self.assertTrue(
                        self._compare_xml_files(xml_file, correct_xml),
                        f"Corrected XML does not match reference.\nGot:\n{open(xml_file).read()}\nExpected:\n{open(correct_xml).read()}",
                    )

                    self.assertTrue(validate_xml_with_xmllint(xml_file))

    def test_cmake_parsing(self):
        """Test the CMake parsing functionality."""
        for package_name in os.listdir(self.examples_dir):
            package_dir = os.path.join(self.examples_dir, package_name, "pkg_correct")
            if not os.path.isdir(package_dir):
                continue

            cmake_file = None
            for fname in os.listdir(package_dir):
                if fname.endswith("CMakeLists.txt"):
                    cmake_file = os.path.join(package_dir, fname)
                    break

            if not cmake_file:
                continue

            main_deps, test_deps = read_deps_from_cmake_file(cmake_file)
            package_xml_file = os.path.join(package_dir, "package.xml")

            try:
                parser = ET.XMLParser(remove_blank_text=True)
                tree = ET.parse(package_xml_file, parser)
                root = tree.getroot()
            except ET.XMLSyntaxError:
                self.fail(f"XML Syntax Error in {package_xml_file}")

            formatter = PackageXmlFormatter()
            xml_build_deps = formatter.retrieve_build_dependencies(root)
            xml_test_deps = formatter.retrieve_test_dependencies(root)

            for dep in main_deps:
                self.assertIn(dep, xml_build_deps, f"Missing build dep: {dep}")
            for dep in test_deps:
                self.assertIn(dep, xml_test_deps, f"Missing test dep: {dep}")


if __name__ == "__main__":
    unittest.main()
