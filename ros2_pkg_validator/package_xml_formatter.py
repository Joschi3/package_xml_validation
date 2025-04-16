import argparse
import os
from lxml import etree as ET
from copy import deepcopy

try:
    from .helpers.logger import get_logger
    from .helpers.rosdep_validator import RosdepValidator
    from .helpers.pkg_xml_formatter import PackageXmlFormatter
except ImportError:
    from helpers.logger import get_logger
    from helpers.rosdep_validator import RosdepValidator
    from helpers.pkg_xml_formatter import PackageXmlFormatter
import subprocess

# Order and min/max occurrences of elements


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


class PackageXmlValidator:
    def __init__(
        self,
        check_only=False,
        check_with_xmllint=False,
        check_rosdeps=True,
        compare_with_cmake=False,
        verbose=False,
    ):
        self.verbose = verbose
        self.check_only = check_only
        self.check_with_xmllint = check_with_xmllint
        self.check_rosdeps = check_rosdeps
        self.compare_with_cmake = compare_with_cmake
        self.logger = get_logger(__name__, level="verbose" if verbose else "normal")
        if self.check_rosdeps:
            self.rosdep_validator = RosdepValidator()
        self.formatter = PackageXmlFormatter(
            check_only=check_only,
            check_with_xmllint=check_with_xmllint,
            verbose=verbose,
        )
        self.encountered_unresolvable_error = False
        print(f"Check ROS dependencies: {self.check_rosdeps}")

        # calculate num checks
        self.num_checks = 6
        self.check_count = 0
        if self.check_rosdeps:
            self.num_checks += 1
        if self.check_with_xmllint:
            self.num_checks += 1
        if self.compare_with_cmake:
            self.num_checks += 1

    def find_package_xml_files(self, paths):
        """Locate all package.xml files within the provided paths."""
        package_xml_files = []
        for path in paths:
            if os.path.isfile(path) and os.path.basename(path) == "package.xml":
                package_xml_files.append(path)
            elif os.path.isdir(path):
                for root, _, files in os.walk(path):
                    if "package.xml" in files:
                        package_xml_files.append(os.path.join(root, "package.xml"))
        return package_xml_files

    def check_for_rosdeps(self, root, xml_file):
        """extract list of rosdeps and check if they are valid"""
        rosdeps = []
        for elem in root:
            if "depend" in elem.tag and elem.text:
                rosdeps.append(elem.text.strip())
        if not rosdeps:
            self.logger.info(f"No ROS dependencies found in {xml_file}.")
            return True
        unresolvable = self.rosdep_validator.check_rosdeps(rosdeps)
        if unresolvable:
            self.logger.error(
                f"Unresolvable ROS dependencies found in {xml_file}: {', '.join(unresolvable)}"
            )
            return False
        return True

    def log_check_result(self, check_name, result):
        """Log the result of a check."""
        if result:
            self.logger.debug(
                f"✅ [{self.check_count}/{self.num_checks}] {check_name} passed."
            )
        else:
            self.logger.error(f"❌ [{self.check_count}/{self.num_checks}] failed.")

    def check_and_format_files(self, package_xml_files):
        """Check and format package.xml files if self.check_only is False.
        Returns is_valid, changed_xml
        """

        all_valid = True
        for xml_file in package_xml_files:
            self.logger.info(f"Processing {xml_file}...")

            if not os.path.exists(xml_file):
                raise FileNotFoundError(f"{xml_file} does not exist.")
            if not os.path.isfile(xml_file):
                raise IsADirectoryError(f"{xml_file} is not a file.")
            try:
                parser = ET.XMLParser()
                tree = ET.parse(xml_file, parser)
                root = tree.getroot()
            except Exception as e:
                self.logger.error(f"Error processing {xml_file}: {e}")
                all_valid = False
                continue

            check = self.formatter.check_for_non_existing_tags(root, xml_file)
            all_valid &= check
            self.encountered_unresolvable_error &= check
            self.log_check_result("Check for invalid tags", check)

            if not self.formatter.check_for_empty_lines(root, xml_file):
                all_valid = False
                self.logger.debug(
                    f"❌ [2/{self.num_checks}] Empty lines found in {xml_file}."
                )
            else:
                self.logger.debug(
                    f"✅ [2/{self.num_checks}] No empty lines found in {xml_file}."
                )

            if not self.formatter.check_for_duplicates(root, xml_file):
                all_valid = False
                self.logger.debug(
                    f"❌ [3/{self.num_checks}] Duplicate elements found in {xml_file}."
                )
            else:
                self.logger.debug(
                    f"✅ [3/{self.num_checks}] No duplicate elements found in {xml_file}."
                )

            if not self.formatter.check_element_occurrences(root, xml_file):
                all_valid = False
                self.logger.error(
                    f"❌ [4/{self.num_checks}] Occurrences of elements in {xml_file} are incorrect."
                )
            else:
                self.logger.debug(
                    f"✅ [4/{self.num_checks}] Occurrences of elements in {xml_file} are correct."
                )

            if not self.formatter.check_element_order(root, xml_file):
                all_valid = False
                self.logger.debug(
                    f"❌ [5/{self.num_checks}] Element order in {xml_file} is incorrect."
                )
            else:
                self.logger.debug(
                    f"✅ [5/{self.num_checks}] Element order in {xml_file} is correct."
                )

            if not self.formatter.check_dependency_order(root, xml_file):
                all_valid = False
                self.logger.debug(
                    f"❌ [6/{self.num_checks}] Dependency order in {xml_file} is incorrect."
                )
            else:
                self.logger.debug(
                    f"✅ [6/{self.num_checks}] Dependency order in {xml_file} is correct."
                )

            if not all_valid and not self.check_only:
                # Write back to file
                tree.write(
                    xml_file, encoding="utf-8", xml_declaration=True, pretty_print=True
                )
            if self.check_rosdeps:
                print(f"Check ROS dependencies in {xml_file}")
                if not self.check_for_rosdeps(root, xml_file):
                    all_valid = False
                    self.encountered_unresolvable_error = True
                    self.logger.debug(
                        f"❌ [7/{self.num_checks}] ROS dependencies in {xml_file} are incorrect."
                    )
                else:
                    self.logger.debug(
                        f"✅ [7/{self.num_checks}] All ROS dependencies in {xml_file} are valid."
                    )
            # if self.formatter.check_with_xmllint:
            #     if not validate_xml_with_xmllint(xml_file):
            #         self.logger.error(f"XML validation failed {xml_file}.")
            #         all_valid = False
            #     else:
            #         self.logger.debug(f"XML validation passed {xml_file}.")

        if not all_valid and self.check_only:
            print(
                "❌ Some `package.xml` files have issues. Please review the messages above. 🛠️"
            )
            return False, True
        elif not all_valid:
            if self.encountered_unresolvable_error:
                print(
                    "⚠️ Some `package.xml` files have unresolvable errors. Please check the logs for details. 🔍"
                )
                return False, True
            else:
                print("✅ Corrected `package.xml` files successfully. 🎉")
                return True, True
        else:
            print("🎉 All `package.xml` files are valid and nicely formatted. 🚀")
            return True, False

    def check_and_format(self, src):
        package_xml_files = self.find_package_xml_files(src)
        if not package_xml_files:
            self.logger.info("No package.xml files found in the provided paths.")
            return
        return self.check_and_format_files(package_xml_files)


def main():
    parser = argparse.ArgumentParser(
        description="Validate and format ROS2 package.xml files."
    )
    parser.add_argument(
        "src", nargs="*", help="List of files or directories to process."
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Only check for errors without correcting.",
    )
    parser.add_argument(
        "--file",
        help="Path to a single XML file to process. If provided, 'src' arguments are ignored.",
    )

    parser.add_argument("--verbose", action="store_true", help="Enable verbose output.")

    parser.add_argument(
        "--check-with-xmllint", action="store_true", help="Check XML with xmllint."
    )
    parser.add_argument(
        "--skip-rosdep-key-validation",
        action="store_true",
        help="Check if rosdeps are valid.",
    )

    args = parser.parse_args()

    formatter = PackageXmlValidator(
        check_only=args.check_only,
        verbose=args.verbose,
        check_with_xmllint=args.check_with_xmllint,
        check_rosdeps=not args.skip_rosdep_key_validation,
    )

    if args.file:
        # Process the one file given via --file
        valid = formatter.check_and_format_files([args.file])
    else:
        # Process whatever is found in src
        valid = formatter.check_and_format(args.src)


if __name__ == "__main__":
    main()
