import argparse
import os
from typing import List
from lxml import etree as ET

try:
    from .helpers.logger import get_logger
    from .helpers.rosdep_validator import RosdepValidator
    from .helpers.pkg_xml_formatter import PackageXmlFormatter
    from .helpers.cmake_parsers import read_deps_from_cmake_file
    from .helpers.find_launch_dependencies import scan_files
except ImportError:
    from helpers.logger import get_logger
    from helpers.rosdep_validator import RosdepValidator
    from helpers.pkg_xml_formatter import PackageXmlFormatter
    from helpers.cmake_parsers import read_deps_from_cmake_file
    from helpers.find_launch_dependencies import scan_files
import subprocess


class PackageXmlValidator:
    def __init__(
        self,
        check_only=False,
        check_with_xmllint=False,
        check_rosdeps=True,
        compare_with_cmake=False,
        auto_fill_missing_deps=False,
        path=None,
        verbose=False,
    ):
        self.verbose = verbose
        self.check_only = check_only
        self.check_with_xmllint = check_with_xmllint
        self.check_rosdeps = check_rosdeps
        self.compare_with_cmake = compare_with_cmake
        self.auto_fill_missing_deps = auto_fill_missing_deps
        self.logger = get_logger(__name__, level="verbose" if verbose else "normal")
        if self.check_rosdeps:
            self.rosdep_validator = RosdepValidator(pkg_path=path)
        self.formatter = PackageXmlFormatter(
            check_only=check_only,
            check_with_xmllint=check_with_xmllint,
            verbose=verbose,
        )
        self.encountered_unresolvable_error = False

        # calculate num checks
        self.num_checks = 8
        self.check_count = 1
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
            elif os.path.isfile(path) and os.path.basename(path) == "CMakeLists.txt":
                package_xml_files.append(
                    os.path.join(os.path.dirname(path), "package.xml")
                )
            elif os.path.isdir(path):
                for root, _, files in os.walk(path):
                    if "package.xml" in files:
                        package_xml_files.append(os.path.join(root, "package.xml"))
        # Filter out duplicates
        package_xml_files = list(set(package_xml_files))
        return package_xml_files

    def check_for_rosdeps(self, rosdeps: List[str], xml_file: str):
        """extract list of rosdeps and check if they are valid"""
        if not rosdeps:
            self.logger.info(f"No ROS dependencies found in {xml_file}.")
            return True
        unresolvable = self.rosdep_validator.check_rosdeps_and_local_pkgs(rosdeps)
        pkg_name = os.path.basename(os.path.dirname(xml_file))
        if unresolvable:
            self.logger.error(
                f"Unresolvable ROS dependencies found in {pkg_name}/package.xml: {', '.join(unresolvable)}"
            )
            return False
        return True

    def check_for_cmake(
        self, build_deps: List[str], test_deps: List[str], xml_file: str, root
    ):
        cmake_file = os.path.join(os.path.dirname(xml_file), "CMakeLists.txt")
        if not os.path.exists(cmake_file):
            self.logger.error(
                f"Cannot check for CMake dependencies, {cmake_file} does not exist."
            )
            return False
        pkg_name = os.path.basename(os.path.dirname(xml_file))
        valid_xml = True
        build_deps_cmake, test_deps_cmake = read_deps_from_cmake_file(cmake_file)
        # make sure that all cmake dependencies are in the package.xml if they can be resolved
        unresolvable = self.rosdep_validator.check_rosdeps_and_local_pkgs(
            build_deps_cmake
        )
        missing_deps = [
            dep
            for dep in build_deps_cmake
            if dep not in unresolvable and dep not in build_deps
        ]
        separator = "\n\t\t"
        if missing_deps:
            valid_xml = False
            deps = separator.join(missing_deps)
            if self.check_only or not self.auto_fill_missing_deps:
                self.logger.error(
                    f"Missing dependencies in {pkg_name}/package.xml compared to {pkg_name}/CMakeList.txt: {separator}{deps}"
                )
            else:
                self.logger.warning(
                    f"Auto-filling missing dependencies in {pkg_name}/package.xml: {separator}{deps}"
                )
                self.formatter.add_dependencies(root, missing_deps, "depend")
        unresolvable = self.rosdep_validator.check_rosdeps_and_local_pkgs(
            test_deps_cmake
        )

        missing_deps = [
            dep
            for dep in test_deps_cmake
            if dep not in unresolvable and dep not in test_deps
        ]
        if missing_deps:
            valid_xml = False
            deps = separator.join(missing_deps)
            if self.check_only or not self.auto_fill_missing_deps:
                self.logger.error(
                    f"Missing test dependencies in {pkg_name}/package.xml compared to {pkg_name}/CMakeList.txt: {separator}{deps}"
                )
            else:
                self.logger.warning(
                    f"Auto-filling missing test dependencies in {pkg_name}/package.xml: {separator}{deps}"
                )
                self.formatter.add_dependencies(root, missing_deps, "test_depend")
        return valid_xml

    def validate_xml_with_xmllint(self, xml_file):
        """Validate XML file against the ROS package_format3.xsd schema using xmllint."""
        schema_url = "http://download.ros.org/schema/package_format3.xsd"
        try:
            result = subprocess.run(
                ["xmllint", "--noout", "--schema", schema_url, xml_file],
                capture_output=True,
                text=True,
            )
            if result.returncode != 0:
                self.logger.error(f"XML validation error in {xml_file}:")
                self.logger.error(result.stderr)
                return False
            return True
        except Exception as e:
            self.logger.error(f"Error running xmllint on {xml_file}: {e}")
            return False

    def validate_launch_dependencies(
        self, root, package_xml_file: str, package_name: str, exec_deps: List[str]
    ):
        """Validate launch dependencies in the package.xml file."""

        def extract_launch_deps(folder_names: List[str]) -> List[str]:
            """Extract launch dependencies from the folder names."""
            launch_deps = []
            for folder in folder_names:
                launch_dir = os.path.join(os.path.dirname(package_xml_file), folder)
                if os.path.isdir(launch_dir):
                    launch_deps.extend(scan_files(launch_dir))
            return launch_deps

        launch_folder_names = ["launch", "components"]
        launch_deps = extract_launch_deps(launch_folder_names)
        if not launch_deps:
            self.logger.debug(
                f"No launch dependencies found in {package_name}/package.xml."
            )
            return True

        missing_deps = [
            dep for dep in launch_deps if dep not in exec_deps and dep != package_name
        ]
        if missing_deps:
            sep = "\n\t - "
            self.logger.warning(
                f"Missing launch dependencies in {package_name}/package.xml: {sep}{sep.join(missing_deps)}"
            )

            if self.check_only:
                return False
            else:
                self.logger.info(
                    f"Auto-filling {len(missing_deps)} missing launch dependencies in {package_name}/package.xml."
                )
                # before adding dependencies make sure they are valid rosdeps
                if self.check_rosdeps:
                    invalid_deps = self.rosdep_validator.check_rosdeps_and_local_pkgs(
                        missing_deps
                    )
                    valid_deps = [d for d in missing_deps if d not in invalid_deps]
                    if invalid_deps:
                        self.logger.error(
                            f"Cannot auto-fill invalid launch dependencies: {', '.join(invalid_deps)}"
                        )
                        return False
                    missing_deps = valid_deps
                self.formatter.add_dependencies(root, missing_deps, "exec_depend")
                return False
        return True

    def validate_ament_exports(self, root, xml_file: str):
        """Validate ament_export tags in the package.xml file.
        if a CMakeLists.txt file exists:  <export><build_type>ament_cmake</build_type></export> must be present
        if a setup.py file exists: <export><build_type>ament_python</build_type></export> must be present
        """
        cmake_file = os.path.join(os.path.dirname(xml_file), "CMakeLists.txt")
        setup_file = os.path.join(os.path.dirname(xml_file), "setup.py")
        cmake_present = os.path.exists(cmake_file)
        setup_present = os.path.exists(setup_file)
        export = root.find("export")
        export_exists = export is not None
        build_type = export.find("build_type") if export_exists else None
        build_type_correct = (
            (
                cmake_present
                and build_type is not None
                and build_type.text == "ament_cmake"
            )
            or (
                setup_present
                and build_type is not None
                and build_type.text == "ament_python"
            )
            or (not cmake_present and not setup_present)
        )
        if build_type_correct:
            return True
        else:
            if not export_exists:
                self.logger.error(
                    f"Missing <export> tag in package.xml. Please include {'<export><build_type>ament_cmake</build_type></export>' if cmake_present else '<export><build_type>ament_python</build_type></export>'}."
                )
            else:
                self.logger.error(
                    f"Incorrect <build_type> in <export> tag in package.xml. Expected {'ament_cmake' if cmake_present else 'ament_python'}, found {build_type.text if build_type is not None else 'None'}."
                )
        if self.check_only:
            return False
        if not self.auto_fill_missing_deps:
            self.encountered_unresolvable_error = True
            return False
        else:
            self.formatter.add_build_type_export(
                root, "ament_cmake" if cmake_present else "ament_python"
            )
            self.logger.warning(
                f"Auto-filling <export><build_type>{'ament_cmake' if cmake_present else 'ament_python'}</build_type></export> in {os.path.basename(os.path.dirname(xml_file))}/package.xml."
            )
            return False  # Indicate that changes were made

    def log_check_result(self, check_name, result):
        """Log the result of a check."""
        if result:
            self.logger.debug(
                f"✅ [{self.check_count}/{self.num_checks}] {check_name} passed."
            )
        else:
            self.logger.debug(
                f"❌ [{self.check_count}/{self.num_checks}] {check_name} failed."
            )
        self.check_count += 1
        if self.check_count > self.num_checks:
            self.check_count = 1

    def perform_check(self, check_name, check_function, *args):
        """Perform a single check, log the result, and update validation flags."""
        # self.logger.debug(
        #    f"🛠️ [{self.check_count}/{self.num_checks}] Performing {check_name}."
        # )
        result = check_function(*args)
        self.log_check_result(check_name, result)
        self.xml_valid &= result
        return result

    def check_and_format_files(self, package_xml_files):
        """Check and format package.xml files if self.check_only is False.
        Returns is_valid, changed_xml
        """

        self.all_valid = True
        for xml_file in package_xml_files:
            self.xml_valid = True
            pkg_name = os.path.basename(os.path.dirname(xml_file))
            self.logger.info(f"Processing {pkg_name}...")

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
                self.xml_valid = False
                continue

            # Perform xml checks
            valid = self.perform_check(
                "Check for invalid tags",
                self.formatter.check_for_non_existing_tags,
                root,
                xml_file,
            )
            self.encountered_unresolvable_error |= not valid
            self.perform_check(
                "Check for empty lines",
                self.formatter.check_for_empty_lines,
                root,
                xml_file,
            )
            self.perform_check(
                "Check for duplicate elements",
                self.formatter.check_for_duplicates,
                root,
                xml_file,
            )
            self.perform_check(
                "Check element occurrences",
                self.formatter.check_element_occurrences,
                root,
                xml_file,
            )
            self.perform_check(
                "Check element order",
                self.formatter.check_element_order,
                root,
                xml_file,
            )
            self.perform_check(
                "Check dependency order",
                self.formatter.check_dependency_order,
                root,
                xml_file,
            )

            self.perform_check(
                "Check launch dependencies",
                self.validate_launch_dependencies,
                root,
                xml_file,
                self.formatter.get_package_name(root),
                self.formatter.retrieve_exec_dependencies(root),
            )

            self.perform_check(
                "Check build type export",
                self.validate_ament_exports,
                root,
                xml_file,
            )

            # Check rosdeps if enabled
            if self.check_rosdeps:
                rosdeps = self.formatter.retrieve_all_dependencies(root)
                valid = self.perform_check(
                    "Check ROS dependencies", self.check_for_rosdeps, rosdeps, xml_file
                )
                self.encountered_unresolvable_error |= not valid
            # Check with xmllint if enabled
            if self.check_with_xmllint:
                valid = self.perform_check(
                    "Check with xmllint", self.validate_xml_with_xmllint, xml_file
                )
                self.encountered_unresolvable_error |= not valid
            # Check for CMake dependencies if enabled
            if self.compare_with_cmake:
                build_deps = self.formatter.retrieve_build_dependencies(root)
                test_deps = self.formatter.retrieve_test_dependencies(root)
                valid = self.perform_check(
                    "Check CMake dependencies",
                    self.check_for_cmake,
                    build_deps,
                    test_deps,
                    xml_file,
                    root,
                )
                if not self.auto_fill_missing_deps:
                    self.encountered_unresolvable_error |= not valid

            # Write back to file if not in check-only mode
            if not self.xml_valid and not self.check_only:
                # ET.indent(root, space="  ")
                tree.write(
                    xml_file, encoding="utf-8", xml_declaration=True, pretty_print=True
                )

            self.all_valid &= self.xml_valid
        # Final result messages
        if not self.all_valid and self.check_only:
            self.logger.warning(
                "❌ Some `package.xml` files have issues. Please review the messages above. 🛠️"
            )
            return False
        elif not self.all_valid:
            if self.encountered_unresolvable_error:
                self.logger.warning(
                    "⚠️ Some `package.xml` files have unresolvable errors. Please check the logs for details. 🔍"
                )
                return False
            else:
                self.logger.info("✅ Corrected `package.xml` files successfully. 🎉")
                return False
        else:
            self.logger.info(
                "🎉 All `package.xml` files are valid and nicely formatted. 🚀"
            )
            return True

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
        "--check-with-xmllint",
        action="store_true",
        help="Recheck XML schema using xmllint.",
    )
    parser.add_argument(
        "--skip-rosdep-key-validation",
        action="store_true",
        help="Check if rosdeps are valid.",
    )

    parser.add_argument(
        "--compare-with-cmake",
        action="store_true",
        help="Check if all CMake dependencies are in package.xml.",
    )

    parser.add_argument(
        "--auto-fill-missing-deps",
        action="store_true",
        help="Automatically fill missing dependencies in package.xml [--compare-with-cmake must be set].",
    )

    args = parser.parse_args()

    # if file not given and src is empty, assume current directory
    if not args.file and not args.src:
        args.src = [os.getcwd()]

    # if env var ROS_DISTRO not available, force skip rosdep key validation
    if not args.skip_rosdep_key_validation and "ROS_DISTRO" not in os.environ:
        args.skip_rosdep_key_validation = True
        print(
            "ROS_DISTRO environment variable not set. Skipping rosdep key validation."
        )

    # if --skip-rosdep-key-validation is set -> compare with cmake and auto-fill missing deps are not possible
    if args.skip_rosdep_key_validation and args.compare_with_cmake:
        print("Cannot use --compare-with-cmake with --skip-rosdep-key-validation.")
        args.compare_with_cmake = False

    # --auto-fill-missing-deps is only possible with --compare-with-cmake
    if not args.compare_with_cmake and args.auto_fill_missing_deps:
        print("Cannot use --auto-fill-missing-deps without --compare-with-cmake.")
        args.auto_fill_missing_deps = False

    formatter = PackageXmlValidator(
        check_only=args.check_only,
        verbose=args.verbose,
        check_with_xmllint=args.check_with_xmllint,
        check_rosdeps=not args.skip_rosdep_key_validation,
        compare_with_cmake=args.compare_with_cmake,
        auto_fill_missing_deps=args.auto_fill_missing_deps,
        path=args.file if args.file else args.src[0],
    )

    if args.file:
        # Process the one file given via --file
        valid = formatter.check_and_format_files([args.file])
    else:
        # Process whatever is found in src
        valid = formatter.check_and_format(args.src)
    if not valid:
        exit(1)


if __name__ == "__main__":
    main()
