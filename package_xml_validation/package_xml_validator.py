#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

from __future__ import annotations

import argparse
import os
from typing import TYPE_CHECKING
from collections.abc import Iterable

import argcomplete
import lxml.etree as ET

from .helpers.cmake_parsers import _DEFAULT_CMAKE_KEYS_NO_ROSDEP
from .helpers.exception_parser import DependencyExceptions, parse_exceptions
from .helpers.logger import get_logger
from .helpers.pkg_xml_formatter import PackageXmlFormatter
from .helpers.rosdep_validator import RosdepValidator
from .helpers.validation_steps import (
    BuildToolDependStep,
    BuildTypeExportStep,
    CMakeComparisonStep,
    DependencyExclusivityStep,
    FormatterValidationStep,
    LaunchDependencyStep,
    ManifestSchemaStep,
    MemberOfGroupStep,
    RosdepCheckStep,
    ValidationConfig,
    ValidationStep,
)
from .helpers.workspace import find_package_xml_files

if TYPE_CHECKING:
    from .helpers.package_types import XmlElement


class PackageXmlValidator:
    def __init__(
        self,
        check_only: bool = False,
        check_rosdeps: bool = True,
        compare_with_cmake: bool = False,
        auto_fill_missing_deps: bool = False,
        strict_cmake_checking: bool = False,
        missing_deps_only: bool = False,
        ignore_formatting_errors: bool = False,
        path: str | None = None,
        verbose: bool = False,
        cmake_keys_no_rosdep: Iterable[str] | None = None,
        ignored_deps: Iterable[str] | None = None,
        skip_launch_dep_check: bool = False,
        evaluate_conditions: bool = True,
    ) -> None:
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
            cmake_keys_no_rosdep: Additional CMake `find_package` names that do
                not require a `package.xml` `<depend>` entry. Merged with the
                built-in defaults; never replaces them.
            ignored_deps: Global set of dependency names to ignore in validation.
            skip_launch_dep_check: Skip launch and test file dependency checks.
            evaluate_conditions: Honour REP-149 ``condition="…"`` attributes on
                dependency tags. When True (default), entries whose condition
                evaluates to False against ``os.environ`` are skipped during
                rosdep / CMake comparison. Disable with ``--ignore-conditions``
                to evaluate every entry regardless.

        Returns:
            None.

        """
        self.verbose = verbose
        self.global_ignored_deps = (
            frozenset(ignored_deps) if ignored_deps else frozenset()
        )
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
        self.rosdep_validator: RosdepValidator | None = None
        if self.check_rosdeps:
            self.rosdep_validator = RosdepValidator(pkg_path=path)
        self.formatter = PackageXmlFormatter(
            check_only=self.check_only,
            verbose=verbose,
        )
        effective_cmake_keys = _DEFAULT_CMAKE_KEYS_NO_ROSDEP | frozenset(
            cmake_keys_no_rosdep or ()
        )
        self.validation_config = ValidationConfig(
            check_only=self.check_only,
            auto_fill_missing_deps=self.auto_fill_missing_deps,
            check_rosdeps=self.check_rosdeps,
            compare_with_cmake=self.compare_with_cmake,
            strict_cmake_checking=self.strict_cmake_checking,
            missing_deps_only=self.missing_deps_only,
            ignore_formatting_errors=self.ignore_formatting_errors,
            cmake_keys_no_rosdep=effective_cmake_keys,
            skip_launch_dep_check=skip_launch_dep_check,
            evaluate_conditions=evaluate_conditions,
        )

        # num_checks and check_count are set per-file in check_and_format_files
        # once the actual step list has been built.
        self.num_checks = 0
        self.check_count = 1

    def log_check_result(self, check_name: str, result: bool) -> None:
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

    def _build_steps(
        self, root: XmlElement, xml_file: str, package_name: str | None
    ) -> list[ValidationStep]:
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

        # Parse per-package exceptions from XML comments and merge with global
        exceptions = parse_exceptions(root)
        if self.global_ignored_deps:
            merged = exceptions.ignored_deps | self.global_ignored_deps
            exceptions = DependencyExceptions(ignored_deps=merged)

        steps: list[ValidationStep] = [
            ManifestSchemaStep(self.validation_config),
            FormatterValidationStep(self.validation_config, self.formatter),
            LaunchDependencyStep(
                self.validation_config,
                self.formatter,
                self.rosdep_validator if self.check_rosdeps else None,
                package_name,
                exec_deps,
                test_deps,
                exceptions,
            ),
        ]

        if not self.missing_deps_only:
            steps.extend(
                [
                    DependencyExclusivityStep(self.validation_config),
                    BuildToolDependStep(self.validation_config, self.formatter),
                    MemberOfGroupStep(self.validation_config, self.formatter),
                    BuildTypeExportStep(self.validation_config, self.formatter),
                ]
            )

        rosdep_validator = self.rosdep_validator
        if (
            self.check_rosdeps
            and not self.missing_deps_only
            and rosdep_validator is not None
        ):
            steps.append(
                RosdepCheckStep(
                    self.validation_config, self.formatter, rosdep_validator
                )
            )

        if self.compare_with_cmake and rosdep_validator is not None:
            steps.append(
                CMakeComparisonStep(
                    self.validation_config,
                    self.formatter,
                    rosdep_validator,
                    exceptions,
                )
            )

        return steps

    def check_and_format_files(self, package_xml_files: Iterable[str]) -> bool:
        """Validate and optionally format a list of package.xml files.

        Args:
            package_xml_files: Iterable of package.xml file paths.

        Returns:
            True if all files are valid, otherwise False.

        """
        self.all_valid = True
        encountered_unresolvable_error = False
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
                # Explicit hardening against XXE: disable network access for
                # external entities and skip entity resolution entirely.
                # lxml 5.x already defaults to no_network=True and
                # resolve_entities='internal', but stating it at the call
                # site documents intent and survives future lxml/parser swaps.
                parser = ET.XMLParser(no_network=True, resolve_entities=False)
                tree = ET.parse(xml_file, parser)
                root = tree.getroot()
            except (ET.XMLSyntaxError, OSError) as e:
                self.logger.error(f"Error parsing {xml_file}: {e}")
                self.xml_valid = False
                self.all_valid = False
                continue

            package_name = self.formatter.get_package_name(root)
            steps = self._build_steps(root, xml_file, package_name)
            self.num_checks = len(steps)
            self.check_count = 1

            xml_changed = False
            for step in steps:
                result = step.perform_check(root, xml_file)
                root = result.root
                # Invariant: any step that sets `changed=True` also sets
                # `valid=False`, so AND-ing with `valid` is sufficient.
                self.xml_valid &= result.valid
                xml_changed |= result.changed

                for warning in result.warnings:
                    self.logger.warning(warning)
                for error in result.errors:
                    self.logger.error(error)
                for critical in result.critical_errors:
                    self.logger.error(critical)
                    encountered_unresolvable_error = True

                self.log_check_result(step.name, result.valid and not result.changed)

            # Only write when a step actually mutated; an unfixable failure
            # must not dirty the user's tree with a no-op reserialization.
            if xml_changed and not self.check_only:
                # Re-normalize whitespace/indentation in case a mutator
                # (e.g. add_buildtool_depends) left the tree slightly off.
                self.formatter.check_for_empty_lines(root, xml_file)
                self.formatter.check_indentation(root)
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
            if encountered_unresolvable_error:
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

    def check_and_format(self, src: Iterable[str]) -> bool:
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


def main() -> None:
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
        help="Skip verifying that dependency names exist in the rosdep database.",
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

    parser.add_argument(
        "--ignore-cmake-key",
        action="append",
        default=[],
        metavar="KEY",
        help=(
            "CMake find_package name that does not require a package.xml "
            "<depend> entry. May be passed multiple times. Merged with the "
            "built-in defaults (Threads, OpenMP, ament_cmake)."
        ),
    )

    parser.add_argument(
        "--ignore-deps",
        type=str,
        default="",
        help="Comma-separated list of dependency names to globally ignore in validation.",
    )

    parser.add_argument(
        "--skip-launch-dep-check",
        action="store_true",
        help="Skip checking for missing dependencies in launch and test files.",
    )

    parser.add_argument(
        "--ignore-conditions",
        action="store_true",
        help=(
            'Disable evaluation of REP-149 condition="…" attributes. By default '
            "dependencies whose condition evaluates to False against the current "
            "environment are skipped during rosdep / CMake comparison; with this "
            "flag every entry is evaluated regardless of its condition."
        ),
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

    global_ignored = (
        [d.strip() for d in args.ignore_deps.split(",") if d.strip()]
        if args.ignore_deps
        else []
    )

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
        cmake_keys_no_rosdep=args.ignore_cmake_key,
        ignored_deps=global_ignored,
        skip_launch_dep_check=args.skip_launch_dep_check,
        evaluate_conditions=not args.ignore_conditions,
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
