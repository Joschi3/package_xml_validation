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
        cls.formatter = PackageXmlValidator(
            check_only=False,
            verbose=True,
            auto_fill_missing_deps=True,
            check_rosdeps=True,
            compare_with_cmake=True,
        )
        # mock the rosdep validator -> pretend all packages are resolvable
        cls.formatter.rosdep_validator = MagicMock()
        cls.formatter.rosdep_validator.check_rosdeps_and_local_pkgs = MagicMock(
            return_value=[]
        )

        cls.formatter_check_only = PackageXmlValidator(
            check_only=True,
            verbose=True,
            auto_fill_missing_deps=True,
            check_rosdeps=True,
            compare_with_cmake=True,
        )
        # mock the rosdep validator -> pretend all packages are resolvable
        cls.formatter_check_only.rosdep_validator = MagicMock()
        cls.formatter_check_only.rosdep_validator.check_rosdeps_and_local_pkgs = (
            MagicMock(return_value=[])
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

    def test_xml_formatting(self):
        """
        Iterate over all example packages in the test directory,
        """
        example_pkgs = os.listdir(self.examples_dir)
        for example_pkg in example_pkgs:
            correct_xml = os.path.join(
                self.examples_dir, example_pkg, "pkg_correct", "package.xml"
            )
            build_type_dir = os.path.join(self.test_dir, example_pkg)
            for pkg in os.listdir(build_type_dir):
                xml_file = os.path.join(build_type_dir, pkg, "package.xml")

                # Use subTest to continue testing other files even if this one fails
                with self.subTest(example_pkg=example_pkg, pkg=pkg, xml_file=xml_file):
                    # apply the formatter
                    with open(xml_file) as f:
                        xml_content = f.read()

                    # ---------------------- 1. Use check_only formatter ----------------------
                    valid = self.formatter_check_only.check_and_format_files([xml_file])
                    msg = ""

                    if not pkg == "pkg_correct":
                        if valid:
                            with open(xml_file) as f:
                                msg = f"Formatted XML file {xml_file}:\n'{f.read()}'"
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
                    original_xml = os.path.join(
                        self.examples_dir, example_pkg, pkg, "package.xml"
                    )
                    # make sure that the check only formatter did not change the file
                    self.assertTrue(
                        self._compare_xml_files(xml_file, original_xml),
                        f"XML files do not match: {xml_file} != {original_xml}",
                    )

                    # ------------------------- 2. Use full formatter ------------------------
                    valid = self.formatter.check_and_format_files([xml_file])
                    msg = ""

                    if not pkg == "pkg_correct":
                        if valid:
                            with open(xml_file) as f:
                                msg = f"Formatted XML file {xml_file}:\n'{f.read()}'"
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

                    self.assertTrue(
                        self._compare_xml_files(xml_file, correct_xml),
                        f"XML files do not match: {xml_file} != {correct_xml}",
                    )

                    # validate the XML file with xmllint
                    self.assertTrue(
                        validate_xml_with_xmllint(xml_file),
                        f"XML file {xml_file} failed xmllint validation.",
                    )

    def test_cmake_parsing(self):
        """
        Test the CMake parsing functionality.
        Compare that cmake parser returns the same dependencies as package xml parser for correct packages.
        """
        for package_name in os.listdir(self.examples_dir):
            package_dir = os.path.join(self.examples_dir, package_name, "pkg_correct")
            if not os.path.isdir(package_dir):
                continue
            for fname in os.listdir(package_dir):
                if not fname.endswith("CMakeLists.txt"):
                    continue
                cmake_file = os.path.join(package_dir, fname)
                print(f"Testing CMake file: {cmake_file}")
                # Here you would implement the actual CMake parsing and validation logic
                main_deps, test_deps = read_deps_from_cmake_file(cmake_file)

                # read deps from package.xml
                package_xml_file = os.path.join(package_dir, "package.xml")
                self.assertTrue(
                    os.path.exists(package_xml_file),
                    f"Expected package.xml file to exist at {package_xml_file}",
                )
                try:
                    parser = ET.XMLParser()
                    tree = ET.parse(package_xml_file, parser)
                    root = tree.getroot()
                except ET.XMLSyntaxError as e:
                    self.fail(f"XML Syntax Error in {package_xml_file}: {e}")
                formatter = PackageXmlFormatter()
                xml_build_deps = formatter.retrieve_build_dependencies(root)
                xml_test_deps = formatter.retrieve_test_dependencies(root)
                # Compare the dependencies
                for dep in main_deps:
                    self.assertIn(
                        dep,
                        xml_build_deps,
                        f"Dependency {dep} not found in package.xml build dependencies in {package_name}.",
                    )
                for dep in test_deps:
                    self.assertIn(
                        dep,
                        xml_test_deps,
                        f"Dependency {dep} not found in package.xml test dependencies in {package_name}.",
                    )


if __name__ == "__main__":
    # Run the tests
    unittest.main()
