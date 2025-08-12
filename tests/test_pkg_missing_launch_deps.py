import os
import unittest
import tempfile
import shutil
import subprocess
import lxml.etree as ET
from package_xml_validation.package_xml_validator import (
    PackageXmlValidator,
)


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
        cls.formatter = PackageXmlValidator(
            check_only=False,
            verbose=True,
            auto_fill_missing_deps=True,
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
                    valid = self.formatter.check_and_format_files([xml_file])
                    msg = ""

                    if not pkg == "pkg_correct":
                        if valid:
                            with open(xml_file) as f:
                                msg = f"Formatted XML file {xml_file}:\n'{f.read()}'"
                        self.assertFalse(
                            valid,
                            f"XML file {xml_file} is expected to be invalid but was valid. {msg}",
                        )
                    else:
                        if not valid:
                            with open(xml_file) as f:
                                msg = f"Invalid XML file {xml_file}:\n{f.read()}"
                        self.assertTrue(
                            valid,
                            f"XML file {xml_file} is expected to be valid but was invalid. {msg}",
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


if __name__ == "__main__":
    # Run the tests
    unittest.main()
