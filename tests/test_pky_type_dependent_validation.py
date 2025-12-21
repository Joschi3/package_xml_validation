import os
import unittest
import tempfile
import shutil
import subprocess
from unittest.mock import MagicMock
import lxml.etree as ET

from package_xml_validation.package_xml_validator import (
    PackageXmlValidator,
)
from enum import Enum


# enum class Formatter Type
class FormatterType(Enum):
    CHECK_ONLY = "check_only"
    NO_AUTO_FILL = "no_auto_fill"
    FULL = "full"


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
        cls.examples_dir = os.path.join(current_dir, "examples", "export_tag_examples")
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
        for formatter in cls.formatters.values():
            formatter.rosdep_validator = MagicMock()
            formatter.rosdep_validator.check_rosdeps_and_local_pkgs = MagicMock(
                return_value=[]
            )
            # 2.Configure resolution to return the input string
            # This prevents MagicMock objects from polluting dependency lists
            formatter.rosdep_validator.resolve_cmake_dependency.side_effect = (
                lambda x: x
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
                print(f"Contents of {file1}:\n{f1.read()}")
            with open(file2, encoding="utf-8") as f2:
                print(f"Contents of {file2}:\n{f2.read()}")
            quit()
        return result

    def test_xml_formatting(self):
        """
        Iterate over all example packages in the test directory,
        """
        build_types = ["ament_cmake", "msg_pkg", "ament_python"]
        for build_type in build_types:
            correct_xml = os.path.join(
                self.examples_dir, build_type, "pkg_correct", "package.xml"
            )
            build_type_dir = os.path.join(self.test_dir, build_type)
            for pkg in os.listdir(build_type_dir):
                xml_file = os.path.join(build_type_dir, pkg, "package.xml")
                # apply the formatter
                for type, formatter in self.formatters.items():
                    with self.subTest(
                        pkg=pkg,
                        build_type=build_type,
                        formatter_type=type.value,
                    ):
                        # re-copy file from examples_dir to test_dir
                        shutil.copy2(
                            os.path.join(
                                self.examples_dir, build_type, pkg, "package.xml"
                            ),
                            xml_file,
                        )
                        valid = formatter.check_and_format_files([xml_file])
                        if not pkg == "pkg_correct":
                            self.assertFalse(
                                valid,
                                f"XML file {xml_file} is expected to be invalid but was valid. (using formatter: {type.value})",
                            )
                        else:
                            self.assertTrue(
                                valid,
                                f"XML file {xml_file} is expected to be valid but was invalid. (using formatter: {type.value})",
                            )
                        original_xml = os.path.join(
                            self.examples_dir, build_type, pkg, "package.xml"
                        )
                        # only when using the FULL formatter the missing parts should be added
                        expected_xml = (
                            correct_xml if type == FormatterType.FULL else original_xml
                        )
                        self.assertTrue(
                            self._compare_xml_files_and_print(xml_file, expected_xml),
                            f"XML files do not match: {xml_file} != {expected_xml}",
                        )
                        # validate the XML file with xmllint
                        self.assertTrue(
                            validate_xml_with_xmllint(xml_file),
                            f"XML file {xml_file} failed xmllint validation.",
                        )


if __name__ == "__main__":
    # Run the tests
    unittest.main()
