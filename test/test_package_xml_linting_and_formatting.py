import os
import unittest
import tempfile
import shutil
import filecmp
import subprocess

# ------------------------------------------------------------------
# If your code is in xml_formatter.py, you might do:
# from xml_formatter import check_and_format_files, validate_xml_with_xmllint
#
# Otherwise, inline or adjust as needed.
# ------------------------------------------------------------------

from ros2_pkg_validator.package_xml_formatter import check_and_format_files


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


class TestPackageXmlFormatter(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """
        We assume the example files are in 'tests/examples'.
        Adjust the directory path to match your actual setup.
        """
        current_dir = os.path.dirname(__file__)
        cls.examples_dir = os.path.join(current_dir, "examples")

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
        return "_fail.xml" in filename

    def _is_correct_file(self, filename: str) -> bool:
        """
        Returns True if the file name indicates it is an 'original_XX_correct.xml'.
        """
        return "_correct.xml" in filename and "_fail" not in filename

    def test_xml_formatting(self):
        """
        Iterates over all 'original_XX_*.xml' in the temp test directory,
        checks them with and without --check,
        and compares results to expectations and corrected files.
        """
        for fname in os.listdir(self.test_dir):
            if not fname.startswith("original_") or not fname.endswith(".xml"):
                # Skip any non-original test files
                continue

            original_path = os.path.join(self.test_dir, fname)
            print("\nTesting file:", original_path)

            # --- 1) Test with --check -------------------------------------
            # We expect:
            #  - 'original_XX_correct.xml' => returns True (all_valid=True), file unchanged
            #  - 'original_XX_fail.xml'    => returns False (all_valid=False), file unchanged

            with open(original_path, "rb") as f_before:
                original_bytes_before_check = f_before.read()

            all_valid_check = check_and_format_files([original_path], check_only=True)

            if self._is_correct_file(fname):
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

            # Reload the file from disk (in case some other step changed it).
            with open(original_path, "rb") as f_before:
                original_bytes_before_fix = f_before.read()

            all_valid_fix = check_and_format_files([original_path], check_only=False)
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
                # For fail files, we expect check_and_format_files to fix them => all_valid could be True
                # or possibly still False if there's an unfixable error. Usually you'd expect True if it can be fixed.
                # Depending on your logic, it might just do partial fixes. For most typical scenarios, you'd expect True:
                self.assertTrue(
                    all_valid_fix,
                    f"Expected fail file {fname} to be fixed and become valid.",
                )

                # Compare to "corrected_XX.xml" if that file exists
                # We replace 'original_' with 'corrected_' in the filename
                corrected_fname = fname.replace("original_", "corrected_")
                corrected_path = os.path.join(self.test_dir, corrected_fname)
                if os.path.exists(corrected_path):
                    # Compare the final file with corrected_XX.xml
                    self.assertTrue(
                        filecmp.cmp(final_path, corrected_path, shallow=False),
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


if __name__ == "__main__":
    unittest.main()
