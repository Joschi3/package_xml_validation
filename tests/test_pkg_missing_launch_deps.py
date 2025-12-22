import os
import unittest
from unittest.mock import MagicMock
import tempfile
import shutil
import subprocess
import lxml.etree as ET
from package_xml_validation.package_xml_validator import (
    PackageXmlValidator,
)
from enum import Enum


class FormatterType(Enum):
    CHECK_ONLY = "check_only"
    NO_AUTO_FILL = "no_auto_fill"
    FULL = "full"
    ALL_ROSDEPS_ARE_UNRESOLVABLE = "all_rosdeps_are_unresolvable"


#  GOAl
# Check that launch dependencies are correctly added to the package.xml
# CHECK-ONLY -> find missing but DO NOT CHANGE THE FILE
# NO-AUTO-FILL -> find missing and do not autofill
# FULL -> find missing and autofill
# ALL-ROSDEPS-ARE-UNRESOLVABLE -> find missing but cannot autofill due to unresolvable rosdeps


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
        cls.examples_dir = os.path.join(current_dir, "examples", "launch_pkg_examples")
        cls.formatters = {}
        cls.formatters[FormatterType.FULL] = PackageXmlValidator(
            check_only=False,
            verbose=True,
            auto_fill_missing_deps=True,
            check_rosdeps=True,
            compare_with_cmake=True,
        )
        cls.formatters[FormatterType.CHECK_ONLY] = PackageXmlValidator(
            check_only=True,
            verbose=True,
            auto_fill_missing_deps=False,
            check_rosdeps=True,
            compare_with_cmake=False,
        )

        cls.formatters[FormatterType.NO_AUTO_FILL] = PackageXmlValidator(
            check_only=False,
            verbose=True,
            auto_fill_missing_deps=False,
            check_rosdeps=True,
            compare_with_cmake=False,
        )
        cls.formatters[FormatterType.ALL_ROSDEPS_ARE_UNRESOLVABLE] = (
            PackageXmlValidator(
                check_only=False,
                verbose=True,
                auto_fill_missing_deps=True,
                check_rosdeps=True,
                compare_with_cmake=True,
            )
        )
        # Configure Mocks
        for key, formatter in cls.formatters.items():
            formatter.rosdep_validator = MagicMock()

            # --- CRITICAL FIX START ---
            # Ensure resolve_cmake_dependency returns the input string (pass-through).
            # Without this, it returns a MagicMock object, causing TypeError during string join.
            formatter.rosdep_validator.resolve_cmake_dependency.side_effect = (
                lambda x: x
            )

            if key is FormatterType.ALL_ROSDEPS_ARE_UNRESOLVABLE:
                # Simulate that ALL checked dependencies are missing/unresolvable
                formatter.rosdep_validator.check_rosdeps_and_local_pkgs = MagicMock(
                    side_effect=lambda deps: deps
                )
                # Mock "normal" rosdep check to avoid lookup errors
                formatter.check_for_rosdeps = MagicMock(return_value=True)
            else:
                # Simulate that ALL dependencies are resolvable (empty list of missing)
                formatter.rosdep_validator.check_rosdeps_and_local_pkgs = MagicMock(
                    side_effect=lambda deps: []
                )

    def setUp(self):
        """
        Create a temporary directory for each test, copy example files into it,
        so we can safely modify them during tests.
        """
        self.test_dir = tempfile.mkdtemp(prefix="xml_tests_")

        # copy contents of examples_dir to test_dir
        shutil.copytree(self.examples_dir, self.test_dir, dirs_exist_ok=True)

    def tearDown(self):
        """Clean up the temporary directory after each test."""
        shutil.rmtree(self.test_dir)

    def prettyprint(self, element, **kwargs):
        xml = ET.tostring(element, pretty_print=True, **kwargs)
        print(xml.decode(), end="")

    def _compare_xml_files(self, file1: str, file2: str) -> bool:
        """
        Compare two XML files for equality.
        Using xml parser to ignore whitespace and comments.
        file1: is modified by the formatter
        file2: is the expected corrected file
        """
        try:
            tree1 = ET.parse(file1)
            tree2 = ET.parse(file2)

            # Normalize the XML trees
            root1 = tree1.getroot()
            root2 = tree2.getroot()

            for elem1, elem2 in zip(root1.iter(), root2.iter()):
                # Ignore comments and whitespace
                if ET.iselement(elem1) and ET.iselement(elem2):
                    if elem1.tag != elem2.tag or elem1.text != elem2.text:
                        return False
                elif ET.iselement(elem1) and not ET.iselement(elem2):
                    return False
                elif not ET.iselement(elem1) and ET.iselement(elem2):
                    return False
                # make sure there are no more than 2 \n in the tai
                elif elem2.tail.count("\n") > 2:
                    return False
            return True
        except ET.XMLSyntaxError as e:
            print(f"XML Syntax Error: {e}")
            return False

    def _compare_xml_files_and_print(self, file1: str, file2: str) -> bool:
        result = self._compare_xml_files(file1, file2)
        if not result:
            print(f"XML files do not match: {file1} != {file2}")
            with open(file1, encoding="utf-8") as f1:
                content_1 = f1.readlines()
            with open(file2, encoding="utf-8") as f2:
                content_2 = f2.readlines()
            for line1, line2 in zip(content_1, content_2):
                are_equal = line1 == line2
                if are_equal:
                    print(f"    {line1.rstrip()}")
                else:
                    print(f"->   {line1.rstrip()} != {line2.rstrip()}")
        return result

    def test_xml_formatting(self):
        """
        Iterate over all example packages in the test directory,
        """
        example_pkgs = os.listdir(self.examples_dir)
        for example_pkg in example_pkgs:
            correct_xml = os.path.join(
                self.examples_dir, example_pkg, "pkg_correct", "package.xml"
            )
            pkg_dir = os.path.join(self.test_dir, example_pkg)
            for pkg in os.listdir(pkg_dir):
                xml_file = os.path.join(pkg_dir, pkg, "package.xml")
                for type, formatter in self.formatters.items():
                    # Use subTest to continue testing other files even if this one fails
                    with self.subTest(example_pkg=example_pkg, pkg=pkg, type=type):
                        # recopy file from example to test dir
                        shutil.copy2(
                            os.path.join(
                                self.examples_dir, example_pkg, pkg, "package.xml"
                            ),
                            xml_file,
                        )
                        # apply the formatter
                        with open(xml_file) as f:
                            xml_content = f.read()
                        valid = formatter.check_and_format_files([xml_file])
                        msg = ""
                        # pkgs with faulty/missing launch dependencies or unresolvable rosdeps
                        if (
                            not pkg == "pkg_correct"
                            or type == FormatterType.ALL_ROSDEPS_ARE_UNRESOLVABLE
                        ):
                            if valid:
                                with open(xml_file) as f:
                                    msg = (
                                        f"Formatted XML file {xml_file}:\n'{f.read()}'"
                                    )
                            self.assertFalse(
                                valid,
                                f"XML file {xml_file} is expected to be invalid but was valid. {msg} \n vs original: \n{xml_content}",
                            )

                        else:
                            if not valid:
                                with open(xml_file) as f:
                                    msg = f"Invalid XML file {xml_file}:\n{f.read()}"
                            self.assertTrue(
                                valid,
                                f"XML file {xml_file} is expected to be valid but was invalid. {msg} \n vs original: \n{xml_content}",
                            )
                            self._compare_xml_files_and_print(xml_file, correct_xml)
                        original_xml = os.path.join(
                            self.examples_dir, example_pkg, pkg, "package.xml"
                        )
                        # only when using the FULL formatter the missing parts should be added
                        expected_xml = (
                            correct_xml if type == FormatterType.FULL else original_xml
                        )
                        self.assertTrue(
                            self._compare_xml_files_and_print(xml_file, expected_xml),
                            f"XML files do not match: {xml_file} != {expected_xml} (using formatter type: {type})",
                        )

                        # validate the XML file with xmllint
                        self.assertTrue(
                            validate_xml_with_xmllint(xml_file),
                            f"XML file {xml_file} failed xmllint validation.",
                        )


if __name__ == "__main__":
    # Run the tests
    unittest.main()
