import os
import unittest
import tempfile
import shutil
import subprocess
import lxml.etree as ET

from package_xml_validation.package_xml_validator import (
    PackageXmlValidator,
    RosdepValidator,
    read_deps_from_cmake_file,
    PackageXmlFormatter,
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
        cls.examples_dir = os.path.join(current_dir, "examples", "package_xml_examples")

    def setUp(self):
        """
        Create a temporary directory for each test, copy example files into it,
        so we can safely modify them during tests.
        """
        self.test_dir = tempfile.mkdtemp(prefix="xml_tests_")

        for fname in os.listdir(self.examples_dir):
            if fname.startswith("original_") and fname.endswith(".xml"):
                src = os.path.join(self.examples_dir, fname)
                dst = os.path.join(self.test_dir, fname)
                shutil.copy2(src, dst)

            # Also copy the 'corrected_XX.xml' if it exists
            if fname.startswith("corrected_") and fname.endswith(".xml"):
                src = os.path.join(self.examples_dir, fname)
                dst = os.path.join(self.test_dir, fname)
                shutil.copy2(src, dst)

    def tearDown(self):
        """Clean up the temporary directory after each test."""
        shutil.rmtree(self.test_dir)

    def _is_fail_file(self, filename: str) -> bool:
        """
        Returns True if the file name indicates it is an 'original_XX_fail.xml'.
        """
        return filename.endswith("_fail.xml") and filename.startswith("original")

    def _is_correct_file(self, filename: str) -> bool:
        """
        Returns True if the file name indicates it is an 'original_XX_correct.xml'.
        """
        return filename.endswith("_correct.xml") or filename.startswith("corrected_")

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
        Iterates over all 'original_XX_*.xml' in the temp test directory,
        checks them with and without --check,
        and compares results to expectations and corrected files.
        """
        for fname in os.listdir(self.test_dir):
            if (
                not fname.startswith("original_")
                and not fname.startswith("corrected_")
                and not fname.endswith(".xml")
            ):
                # Skip any non-original test files
                continue

            original_path = os.path.join(self.test_dir, fname)
            print("\nTesting file:", original_path)

            # --- 1) Test with --check -------------------------------------
            # We expect:
            #  - 'original_XX_correct.xml' => returns True (all_valid=True), file unchanged
            #  - 'original_XX_fail.xml'    => returns False (all_valid=False), file unchanged

            formatter = PackageXmlValidator(
                check_only=True, verbose=True, check_rosdeps=False
            )
            with open(original_path, "rb") as f_before:
                original_bytes_before_check = f_before.read()

            all_valid_check = formatter.check_and_format_files([original_path])

            if self._is_correct_file(fname):
                if not all_valid_check:
                    with open(original_path, "r") as f_after:
                        print(f"File content after check:\n{f_after.read()}")
                self.assertTrue(
                    all_valid_check,
                    f"Expected correct file {fname} to pass in check-only mode.",
                )
            elif self._is_fail_file(fname):
                self.assertFalse(
                    all_valid_check,
                    f"Expected fail file {fname} to fail in check-only mode.",
                )

            # Confirm no changes were made when using --check
            with open(original_path, "rb") as f_after:
                original_bytes_after_check = f_after.read()
            self.assertEqual(
                original_bytes_before_check,
                original_bytes_after_check,
                f"File {fname} should not have been modified in check-only mode.",
            )

            # --- 2) Test without --check (check_only=False) ---------------
            #  - For correct files => no changes
            #  - For fail   files => corrected to match `corrected_XX.xml`
            # Then we verify the final result is valid with xmllint.

            formatter = PackageXmlValidator(
                check_only=False, verbose=True, check_rosdeps=False
            )

            # Reload the file from disk (in case some other step changed it).
            with open(original_path, "rb") as f_before:
                original_bytes_before_fix = f_before.read()

            all_valid_fix = formatter.check_and_format_files([original_path])
            self.assertTrue(
                isinstance(all_valid_fix, bool),
                "check_and_format_files should return a boolean.",
            )

            # Now check the final contents.
            final_path = original_path  # same file, presumably overwritten if needed
            self.assertTrue(
                os.path.exists(final_path),
                "Expected the package.xml file to still exist after formatting.",
            )

            if self._is_correct_file(fname):
                # For correct files, it should remain correct => all_valid=True
                self.assertTrue(
                    all_valid_fix, f"Expected correct file {fname} to pass when fixing."
                )
                # And the file should remain unchanged
                with open(final_path, "rb") as f_final:
                    final_bytes = f_final.read()
                self.assertEqual(
                    original_bytes_before_fix,
                    final_bytes,
                    f"Correct file {fname} should remain unchanged after fix.",
                )
            elif self._is_fail_file(fname):
                # For fail files, we expect check_and_format_files to fix them => all_valid False since something was fixed
                self.assertFalse(
                    all_valid_fix,
                    f"Expected fail file {fname} to be fixed and return false to indicate that something was fixed.",
                )

                # Compare to "corrected_XX.xml" if that file exists
                # We replace 'original_' with 'corrected_' in the filename
                corrected_fname = fname.replace("original_", "corrected_")
                corrected_path = os.path.join(self.test_dir, corrected_fname)
                if os.path.exists(corrected_path):
                    # Compare the final file with corrected_XX.xml
                    comparison = self._compare_xml_files(final_path, corrected_path)
                    if not comparison:
                        # print corrected file to console
                        with open(final_path, "r") as f_corrected:
                            corrected_content = f_corrected.read()
                        print(f"Corrected file content:\n{corrected_content}")
                    self.assertTrue(
                        comparison,
                        f"File {fname} after fixing does not match {corrected_fname}",
                    )

                else:
                    self.fail(
                        f"Missing corrected file {corrected_fname} in test directory."
                    )

            # --- 3) Validate the final file with xmllint + package_format3.xsd ----
            # We use the provided helper to ensure the final result is schema-valid.
            self.assertTrue(
                validate_xml_with_xmllint(final_path),
                f"The final file {fname} is not valid against package_format3.xsd.",
            )

    def test_rosdep_checking(self):
        """
        Test the rosdep checking functionality.
        This is a placeholder for the actual implementation.
        """
        # test if ros installed by testing whether the environment variable ROS_DISTRO is set
        # if "ROS_DISTRO" not in os.environ:
        #    self.skipTest("ROS is not installed, skipping rosdep checking test.")
        # Example dependencies to check
        dependencies = ["rclcpp", "nonexistent_dependency", "sensor_msgs"]
        validator = RosdepValidator()
        unresolvable = validator.check_rosdeps(dependencies)

        # Check if the expected dependencies are unresolvable
        self.assertIn("nonexistent_dependency", unresolvable)
        self.assertNotIn("rclcpp", unresolvable)
        self.assertNotIn("sensor_msgs", unresolvable)

    def test_integrated_rosdep_checking(self):
        correct_rosdep = os.path.join(self.examples_dir, "no_incorrect_rosdeps.xml")
        incorrect_rosdep = os.path.join(self.examples_dir, "incorrect_rosdeps.xml")
        # Check the correct file
        formatter = PackageXmlValidator(
            check_only=True, verbose=True, check_rosdeps=True
        )
        all_valid_check = formatter.check_and_format_files([correct_rosdep])
        self.assertTrue(
            all_valid_check, "Expected correct file to pass in check-only mode."
        )
        # Check the incorrect file
        all_valid_check = formatter.check_and_format_files([incorrect_rosdep])
        self.assertFalse(
            all_valid_check, "Expected fail file to fail in check-only mode."
        )

    def test_invalid_path(self):
        """
        Test the behavior when an invalid path is provided.
        """
        invalid_path = os.path.join(self.examples_dir, "nonexistent.xml")
        formatter = PackageXmlValidator(check_only=True, verbose=True)
        with self.assertRaises(FileNotFoundError):
            formatter.check_and_format_files([invalid_path])

    def test_cmake_parsing(self):
        """
        Test the CMake parsing functionality.
        This is a placeholder for the actual implementation.
        """
        # iterate cmakes files in examples_dir/cmakes/<package_name>
        cmakes_dir = os.path.join(self.examples_dir, "cmakes")
        for package_name in os.listdir(cmakes_dir):
            package_dir = os.path.join(cmakes_dir, package_name)
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
                package_xml_file = os.path.join(
                    self.examples_dir, "cmakes", package_name, "package.xml"
                )
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
    unittest.main()
