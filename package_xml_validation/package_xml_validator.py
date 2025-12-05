import argparse
import os
import lxml.etree as ET
from enum import Enum
import re

try:
    from .helpers.logger import get_logger
    from .helpers.rosdep_validator import RosdepValidator
    from .helpers.pkg_xml_formatter import PackageXmlFormatter
    from .helpers.cmake_parsers import read_deps_from_cmake_file
    from .helpers.find_launch_dependencies import scan_files
    from .helpers.workspace import find_package_xml_files
except ImportError:
    from helpers.logger import get_logger
    from helpers.rosdep_validator import RosdepValidator
    from helpers.pkg_xml_formatter import PackageXmlFormatter
    from helpers.cmake_parsers import read_deps_from_cmake_file
    from helpers.find_launch_dependencies import scan_files
    from helpers.workspace import find_package_xml_files


class PackageType(Enum):
    CMAKE_PKG = "ament_cmake"
    PYTHON_PKG = "ament_python"
    MSG_PKG = "rosidl_default_generators"


class PackageXmlValidator:
    def __init__(
        self,
        check_only=False,
        check_rosdeps=True,
        compare_with_cmake=False,
        auto_fill_missing_deps=False,
        missing_deps_only=False,
        ignore_formatting_errors=False,
        path=None,
        verbose=False,
    ):
        self.verbose = verbose
        self.missing_deps_only = missing_deps_only
        self.ignore_formatting_errors = ignore_formatting_errors
        self.check_only = check_only or missing_deps_only or ignore_formatting_errors
        self.check_rosdeps = check_rosdeps
        self.logger = get_logger(__name__, level="verbose" if verbose else "normal")
        self.compare_with_cmake = compare_with_cmake
        if self.compare_with_cmake and not self.check_rosdeps:
            self.logger.warning(
                "Comparing with CMake but not checking ROS dependencies is not supported."
            )
            self.compare_with_cmake = False
        self.auto_fill_missing_deps = auto_fill_missing_deps
        if self.check_rosdeps:
            self.rosdep_validator = RosdepValidator(pkg_path=path)
        self.formatter = PackageXmlFormatter(
            check_only=self.check_only,
            check_with_xmllint=False,
            verbose=verbose,
        )
        self.encountered_unresolvable_error = False

        # calculate num checks
        self.num_checks = self._calculate_num_checks()
        self.check_count = 1

    def _calculate_num_checks(self):
        """Calculate how many checks will run based on configuration."""
        if self.missing_deps_only:
            num_checks = 1  # launch dependency check always runs
            if self.compare_with_cmake:
                num_checks += 1
            return num_checks

        base_checks = 11
        if self.ignore_formatting_errors:
            base_checks -= 6  # Skip formatting-only checks
        if self.check_rosdeps:
            base_checks += 1
        if self.compare_with_cmake:
            base_checks += 1
        return base_checks

    def get_package_type(self, xml_file: str) -> tuple[PackageType, bool]:
        """Determine the package type based on the presence of CMakeLists.txt or setup.py.
        Returns <PackageType, bool> where bool indicates if the package is a message package."""
        cmake_file = os.path.join(os.path.dirname(xml_file), "CMakeLists.txt")
        setup_file = os.path.join(os.path.dirname(xml_file), "setup.py")
        is_msg_pkg = False
        pkg_type = PackageType.CMAKE_PKG
        if os.path.exists(cmake_file):
            # check if rosidl_generate_interfaces is in CMakeLists.txt using regex
            regex = r"rosidl_generate_interfaces\s*\(\s*.*?\)"
            with open(cmake_file) as f:
                content = f.read()
                if re.search(regex, content, re.DOTALL):
                    is_msg_pkg = True
            pkg_type = PackageType.CMAKE_PKG
        elif os.path.exists(setup_file):
            pkg_type = PackageType.PYTHON_PKG
        return pkg_type, is_msg_pkg

    def check_for_rosdeps(self, rosdeps: list[str], xml_file: str):
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
        self, build_deps: list[str], test_deps: list[str], xml_file: str, root
    ):
        pkg_type, _ = self.get_package_type(xml_file)
        if pkg_type != PackageType.CMAKE_PKG:
            self.logger.info(
                "Skipping CMake dependency check for package since it is not a CMake package"
            )
            return True
        cmake_file = os.path.join(os.path.dirname(xml_file), "CMakeLists.txt")
        if not os.path.exists(cmake_file):
            self.logger.error(
                f"Cannot check for CMake dependencies, {cmake_file} does not exist."
            )
            return False
        pkg_name = os.path.basename(os.path.dirname(xml_file))
        valid_xml = True
        build_deps_cmake, test_deps_cmake = read_deps_from_cmake_file(cmake_file)
        # remove "ament_cmake" as it is a buildtool_depend
        if "ament_cmake" in build_deps_cmake:
            build_deps_cmake.remove("ament_cmake")
        if "ament_cmake" in test_deps_cmake:
            test_deps_cmake.remove("ament_cmake")
        # ----------------------------  BUILD DEPENDENCIES ----------------------------
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
        # ----------------------------  TEST DEPENDENCIES ----------------------------
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

    def validate_launch_dependencies(
        self,
        root,
        package_xml_file: str,
        package_name: str,
        exec_deps: list[str],
        test_deps: list[str] = [],
    ):
        """Validate launch dependencies in the package.xml file."""

        def extract_launch_deps(folder_names: list[str]) -> list[str]:
            """Extract launch dependencies from the folder names."""
            launch_deps = []
            for folder in folder_names:
                launch_dir = os.path.join(os.path.dirname(package_xml_file), folder)
                if os.path.isdir(launch_dir):
                    launch_deps.extend(scan_files(launch_dir))
            return launch_deps

        def validate_launch_folders(
            launch_folder_names: list[str], xml_deps: list[str], depend_tag: str
        ) -> bool:
            launch_deps = extract_launch_deps(launch_folder_names)
            if not launch_deps:
                return True

            missing_deps = [
                dep
                for dep in launch_deps
                if dep not in xml_deps and dep != package_name
            ]
            if missing_deps:
                sep = "\n\t - "
                self.logger.warning(
                    f"Missing <{depend_tag}> dependencies in {package_name}/package.xml: {sep}{sep.join(missing_deps)}"
                )
            missing_deps = [
                dep
                for dep in launch_deps
                if dep not in xml_deps and dep != package_name
            ]
            if missing_deps:
                sep = "\n\t - "
                self.logger.warning(
                    f"Missing <{depend_tag}> dependencies in {package_name}/package.xml: {sep}{sep.join(missing_deps)}"
                )

                if self.check_only:
                    return False
                elif not self.auto_fill_missing_deps:
                    self.encountered_unresolvable_error = True
                    self.logger.error(
                        f"Cannot auto-fill missing <{depend_tag}> dependencies: {missing_deps} in {package_name}/package.xml. Please add them manually."
                    )
                    return False
                else:
                    self.logger.info(
                        f"Auto-filling {len(missing_deps)} missing <{depend_tag}> dependencies in {package_name}/package.xml."
                    )
                    # before adding dependencies make sure they are valid rosdeps
                    if self.check_rosdeps:
                        invalid_deps = (
                            self.rosdep_validator.check_rosdeps_and_local_pkgs(
                                missing_deps
                            )
                        )
                        valid_deps = [d for d in missing_deps if d not in invalid_deps]
                        if invalid_deps:
                            self.logger.error(
                                f"Cannot auto-fill invalid launch dependencies: {', '.join(invalid_deps)}"
                            )
                            return False
                        missing_deps = valid_deps
                    self.formatter.add_dependencies(root, missing_deps, depend_tag)
                    return False
            return True

        launch_folder_names = ["launch", "components"]
        launch_deps_valid = validate_launch_folders(
            launch_folder_names, exec_deps, "exec_depend"
        )
        test_deps_valid = validate_launch_folders(["test"], test_deps, "test_depend")
        return launch_deps_valid and test_deps_valid

    def validate_buildtool_depend(self, root, xml_file: str):
        """Validate build_tool depend tags in the package.xml file.
        Note: for interface packages 2 buildtool_depend tags are required: ament_cmake and rosidl_default_generators
        """
        pkg_type, is_msg_pkg = self.get_package_type(xml_file)
        buildtool = root.findall("buildtool_depend")
        buildtool = [tool.text for tool in buildtool]
        is_buildtool_correct = (
            len(buildtool) > 0
            and (
                (
                    pkg_type == PackageType.CMAKE_PKG
                    and PackageType.CMAKE_PKG.value in buildtool
                )
                or (
                    pkg_type == PackageType.PYTHON_PKG
                    and PackageType.PYTHON_PKG.value in buildtool
                )
            )
            and (not is_msg_pkg or PackageType.MSG_PKG.value in buildtool)
        )

        if is_buildtool_correct:
            return True
        else:
            corrected_buildtool_str = f"<buildtool_depend>{'ament_cmake' if pkg_type == PackageType.CMAKE_PKG else 'ament_python'}</buildtool_depend>"
            if is_msg_pkg:
                corrected_buildtool_str += (
                    f" <buildtool_depend>{PackageType.MSG_PKG.value}</buildtool_depend>"
                )
            if len(buildtool) == 0:
                self.logger.error(
                    f"Missing <buildtool_depend> tag in package.xml. Please include {corrected_buildtool_str}."
                )
            else:
                self.logger.error(
                    f"Incorrect <buildtool_depend> in package.xml. Expected {corrected_buildtool_str}, found {buildtool if buildtool is not None else 'None'}."
                )
        if self.check_only:
            return False
        if not self.auto_fill_missing_deps:
            self.logger.error(
                f"Cannot auto-fill missing <buildtool_depend> in {os.path.basename(os.path.dirname(xml_file))}/package.xml. Please add it manually."
            )
            self.encountered_unresolvable_error = True
            return False
        else:
            self.formatter.add_buildtool_depends(
                root,
                [pkg_type.value, PackageType.MSG_PKG.value]
                if is_msg_pkg
                else [pkg_type.value],
            )
            self.logger.warning(
                f"Auto-filling {corrected_buildtool_str} in {os.path.basename(os.path.dirname(xml_file))}/package.xml."
            )
            return False  # Indicate that changes were made

    def validate_ament_exports(self, root, xml_file: str):
        """Validate ament_export tags in the package.xml file."""
        pkg_type, _ = self.get_package_type(xml_file)
        export = root.find("export")
        export_exists = export is not None
        build_type = export.find("build_type") if export_exists else None
        build_type_correct = (
            (
                pkg_type == PackageType.CMAKE_PKG
                and build_type is not None
                and build_type.text == "ament_cmake"
            )
            or (
                pkg_type == PackageType.PYTHON_PKG
                and build_type is not None
                and build_type.text == "ament_python"
            )
            or (
                not pkg_type == PackageType.CMAKE_PKG
                and not pkg_type == PackageType.PYTHON_PKG
            )
        )
        if build_type_correct:
            return True
        else:
            if not export_exists:
                self.logger.error(
                    f"Missing <export> tag in package.xml. Please include {'<export><build_type>ament_cmake</build_type></export>' if pkg_type == PackageType.CMAKE_PKG else '<export><build_type>ament_python</build_type></export>'}."
                )
            else:
                self.logger.error(
                    f"Incorrect <build_type> in <export> tag in package.xml. Expected {'ament_cmake' if pkg_type == PackageType.CMAKE_PKG else 'ament_python'}, found {build_type.text if build_type is not None else 'None'}."
                )
        if self.check_only:
            return False
        if not self.auto_fill_missing_deps:
            self.encountered_unresolvable_error = True
            return False
        else:
            self.formatter.add_build_type_export(
                root,
                "ament_cmake" if pkg_type == PackageType.CMAKE_PKG else "ament_python",
            )
            self.logger.warning(
                f"Auto-filling <export><build_type>{'ament_cmake' if pkg_type == PackageType.CMAKE_PKG else 'ament_python'}</build_type></export> in {os.path.basename(os.path.dirname(xml_file))}/package.xml."
            )
            return False  # Indicate that changes were made

    def validate_member_of_group(self, root, xml_file: str):
        """Validate member_of_group tag in the package.xml file.
        -> interface packages must include <member_of_group>rosidl_interface_packages</member_of_group>
        """
        member_of_group = root.find("member_of_group")
        _, is_msg_pkg = self.get_package_type(xml_file)
        if is_msg_pkg and (
            member_of_group is None
            or member_of_group.text != "rosidl_interface_packages"
        ):
            self.logger.error(
                f"Missing or incorrect <member_of_group> in package.xml. Expected <member_of_group>rosidl_interface_packages</member_of_group>, found {member_of_group.text if member_of_group is not None else 'None'}."
            )
            if self.check_only:
                return False
            if not self.auto_fill_missing_deps:
                self.encountered_unresolvable_error = True
                return False
            else:
                self.formatter.add_member_of_group(root, "rosidl_interface_packages")
                self.logger.warning(
                    f"Auto-filling <member_of_group>rosidl_interface_packages</member_of_group> in {os.path.basename(os.path.dirname(xml_file))}/package.xml."
                )
                return False  # Indicate that changes were made
        return True

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

    def _build_checks(self, root, xml_file, package_name):
        """Prepare the list of checks to execute for a given package.xml file."""
        checks = []
        exec_deps = self.formatter.retrieve_exec_dependencies(root)
        test_deps = self.formatter.retrieve_test_dependencies(root)

        formatting_checks = [
            (
                "Check for empty lines",
                self.formatter.check_for_empty_lines,
                (root, xml_file),
                False,
            ),
            (
                "Check for duplicate elements",
                self.formatter.check_for_duplicates,
                (root, xml_file),
                False,
            ),
            (
                "Check element occurrences",
                self.formatter.check_element_occurrences,
                (root, xml_file),
                False,
            ),
            (
                "Check element order",
                self.formatter.check_element_order,
                (root, xml_file),
                False,
            ),
            (
                "Check dependency order",
                self.formatter.check_dependency_order,
                (root, xml_file),
                False,
            ),
            (
                "Check indentation",
                self.formatter.check_indentation,
                (root,),
                False,
            ),
        ]

        if not self.missing_deps_only:
            checks.append(
                (
                    "Check for invalid tags",
                    self.formatter.check_for_non_existing_tags,
                    (root, xml_file),
                    True,
                )
            )

            if not self.ignore_formatting_errors:
                checks.extend(formatting_checks)

        checks.append(
            (
                "Check launch dependencies",
                self.validate_launch_dependencies,
                (root, xml_file, package_name, exec_deps, test_deps),
                False,
            )
        )

        if not self.missing_deps_only:
            checks.extend(
                [
                    (
                        "Check build tool depend",
                        self.validate_buildtool_depend,
                        (root, xml_file),
                        False,
                    ),
                    (
                        "Check member of group",
                        self.validate_member_of_group,
                        (root, xml_file),
                        False,
                    ),
                    (
                        "Check build type export",
                        self.validate_ament_exports,
                        (root, xml_file),
                        False,
                    ),
                ]
            )

        if self.check_rosdeps and not self.missing_deps_only:
            rosdeps = self.formatter.retrieve_all_dependencies(root)
            checks.append(
                (
                    "Check ROS dependencies",
                    self.check_for_rosdeps,
                    (rosdeps, xml_file),
                    True,
                )
            )

        if self.compare_with_cmake:
            build_deps = self.formatter.retrieve_build_dependencies(root)
            checks.append(
                (
                    "Check CMake dependencies",
                    self.check_for_cmake,
                    (build_deps, test_deps, xml_file, root),
                    not self.auto_fill_missing_deps,
                )
            )

        return checks

    def check_and_format_files(self, package_xml_files):
        """Check and format package.xml files if self.check_only is False.
        Returns is_valid, changed_xml
        """
        self.all_valid = True
        for xml_file in package_xml_files:
            self.xml_valid = True
            pkg_name = os.path.basename(os.path.dirname(xml_file))
            self.logger.info(f"Processing {pkg_name}...")
            self.logger.debug(f"Checking {xml_file}...")

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

            package_name = self.formatter.get_package_name(root)
            checks = self._build_checks(root, xml_file, package_name)
            self.num_checks = len(checks)
            self.check_count = 1

            for check_name, check_fn, args, mark_unresolvable in checks:
                valid = self.perform_check(check_name, check_fn, *args)
                if mark_unresolvable and not valid:
                    self.encountered_unresolvable_error = True

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
        package_xml_files = find_package_xml_files(src)
        if not package_xml_files:
            self.logger.info(
                "No package.xml files found in the provided paths. Nothing to check."
            )
            return True
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

    parser.add_argument(
        "--missing-deps-only",
        action="store_true",
        help="Only report missing dependencies (implies --check-only).",
    )

    parser.add_argument(
        "--ignore-formatting-errors",
        action="store_true",
        help="Ignore formatting-only checks (implies --check-only).",
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

    formatter = PackageXmlValidator(
        check_only=args.check_only
        or args.missing_deps_only
        or args.ignore_formatting_errors,
        verbose=args.verbose,
        check_rosdeps=not args.skip_rosdep_key_validation,
        compare_with_cmake=args.compare_with_cmake,
        auto_fill_missing_deps=args.auto_fill_missing_deps,
        missing_deps_only=args.missing_deps_only,
        ignore_formatting_errors=args.ignore_formatting_errors,
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
