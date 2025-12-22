#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

import argparse
import os
import lxml.etree as ET
import argcomplete

try:
    from .helpers.logger import get_logger
    from .helpers.rosdep_validator import RosdepValidator
    from .helpers.pkg_xml_formatter import PackageXmlFormatter
    from .helpers.validation_steps import (
        ValidationConfig,
        FormatterValidationStep,
        BuildToolDependStep,
        MemberOfGroupStep,
        BuildTypeExportStep,
        RosdepCheckStep,
        CMakeComparisonStep,
        LaunchDependencyStep,
    )
    from .helpers.workspace import find_package_xml_files
except ImportError:
    from helpers.logger import get_logger
    from helpers.rosdep_validator import RosdepValidator
    from helpers.pkg_xml_formatter import PackageXmlFormatter
    from helpers.validation_steps import (
        ValidationConfig,
        FormatterValidationStep,
        BuildToolDependStep,
        MemberOfGroupStep,
        BuildTypeExportStep,
        RosdepCheckStep,
        CMakeComparisonStep,
        LaunchDependencyStep,
    )
    from helpers.workspace import find_package_xml_files


class PackageXmlValidator:
    def __init__(
        self,
        check_only=False,
        check_rosdeps=True,
        compare_with_cmake=False,
        auto_fill_missing_deps=False,
        strict_cmake_checking=False,
        missing_deps_only=False,
        ignore_formatting_errors=False,
        path=None,
        verbose=False,
    ):
        """Initialize the package.xml validator with feature flags.

        Args:
            check_only: If True, only report issues without modifying files.
            check_rosdeps: Whether to validate rosdep keys.
            compare_with_cmake: Whether to compare dependencies with CMakeLists.txt.
            auto_fill_missing_deps: Whether to auto-fill missing dependencies.
            strict_cmake_checking: Treat unresolved CMake deps as errors.
            missing_deps_only: Only report missing dependency checks.
            ignore_formatting_errors: Skip formatting-only checks.
            path: Path used for workspace discovery in rosdep validation.
            verbose: Enable verbose logging.

        Returns:
            None.

        """
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
        self.strict_cmake_checking = strict_cmake_checking
        self.auto_fill_missing_deps = auto_fill_missing_deps
        if self.check_rosdeps:
            self.rosdep_validator = RosdepValidator(pkg_path=path)
        self.formatter = PackageXmlFormatter(
            check_only=self.check_only,
            check_with_xmllint=False,
            verbose=verbose,
        )
        self.validation_config = ValidationConfig(
            check_only=self.check_only,
            auto_fill_missing_deps=self.auto_fill_missing_deps,
            check_rosdeps=self.check_rosdeps,
            compare_with_cmake=self.compare_with_cmake,
            strict_cmake_checking=self.strict_cmake_checking,
            missing_deps_only=self.missing_deps_only,
            ignore_formatting_errors=self.ignore_formatting_errors,
        )
        self.encountered_unresolvable_error = False

        # calculate num checks
        self.num_checks = self._calculate_num_checks()
        self.check_count = 1

    def _calculate_num_checks(self):
        """Calculate the total number of checks based on configuration.

        Args:
            None.

        Returns:
            Number of checks expected to run.

        """
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

    def log_check_result(self, check_name, result):
        """Log the result of a check and advance the counter.

        Args:
            check_name: Human-readable check name.
            result: True if the check passed, otherwise False.

        Returns:
            None.

        """
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

    def _build_steps(self, root, xml_file, package_name):
        """Build the list of validation steps for a given package.

        Args:
            root: XML root element.
            xml_file: Path to the XML file.
            package_name: Name of the package being validated.

        Returns:
            List of ValidationStep instances to run.

        """
        exec_deps = self.formatter.retrieve_exec_dependencies(root)
        test_deps = self.formatter.retrieve_test_dependencies(root)

        steps = [
            FormatterValidationStep(self.validation_config, self.formatter),
            LaunchDependencyStep(
                self.validation_config,
                self.formatter,
                self.rosdep_validator if self.check_rosdeps else None,
                package_name,
                exec_deps,
                test_deps,
            ),
        ]

        if not self.missing_deps_only:
            steps.extend(
                [
                    BuildToolDependStep(self.validation_config, self.formatter),
                    MemberOfGroupStep(self.validation_config, self.formatter),
                    BuildTypeExportStep(self.validation_config, self.formatter),
                ]
            )

        if self.check_rosdeps and not self.missing_deps_only:
            steps.append(
                RosdepCheckStep(
                    self.validation_config, self.formatter, self.rosdep_validator
                )
            )

        if self.compare_with_cmake:
            steps.append(
                CMakeComparisonStep(
                    self.validation_config, self.formatter, self.rosdep_validator
                )
            )

        return steps

    def check_and_format_files(self, package_xml_files):
        """Validate and optionally format a list of package.xml files.

        Args:
            package_xml_files: Iterable of package.xml file paths.

        Returns:
            True if all files are valid, otherwise False.

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
            steps = self._build_steps(root, xml_file, package_name)
            self.num_checks = len(steps)
            self.check_count = 1

            for step in steps:
                result = step.perform_check(root, xml_file)
                root = result.root
                self.xml_valid &= result.valid
                if result.changed:
                    self.xml_valid = False

                for warning in result.warnings:
                    self.logger.warning(warning)
                for error in result.errors:
                    self.logger.error(error)
                for critical in result.critical_errors:
                    self.logger.error(critical)
                    self.encountered_unresolvable_error = True

                self.log_check_result(step.name, result.valid and not result.changed)

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
        """Find package.xml files under a path and validate them.

        Args:
            src: List of files or directories to scan.

        Returns:
            True if all files are valid, otherwise False.

        """
        package_xml_files = find_package_xml_files(src)
        if not package_xml_files:
            self.logger.info(
                "No package.xml files found in the provided paths. Nothing to check."
            )
            return True
        return self.check_and_format_files(package_xml_files)


def main():
    """CLI entrypoint for package.xml validation.

    Args:
        None.

    Returns:
        None.

    """
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

    parser.add_argument(
        "--strict-cmake-checking",
        action="store_true",
        help="Treat unresolved CMake dependencies as errors instead of warnings.",
    )

    argcomplete.autocomplete(parser)
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
        strict_cmake_checking=args.strict_cmake_checking,
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
