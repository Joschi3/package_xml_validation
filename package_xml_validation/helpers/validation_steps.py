import os
from dataclasses import dataclass, field
from typing import Callable

from .cmake_parsers import read_deps_from_cmake_file
from .find_launch_dependencies import scan_files
from .package_types import PackageType, get_package_type


@dataclass(frozen=True)
class ValidationConfig:
    check_only: bool
    auto_fill_missing_deps: bool
    check_rosdeps: bool
    compare_with_cmake: bool
    strict_cmake_checking: bool
    missing_deps_only: bool
    ignore_formatting_errors: bool


@dataclass
class ValidationResult:
    root: object
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    critical_errors: list[str] = field(default_factory=list)
    changed: bool = False
    valid: bool = True


class ValidationStep:
    name = "Validation step"

    def __init__(self, config: ValidationConfig):
        """Initialize a validation step with configuration.

        Args:
            config: ValidationConfig with feature flags.

        Returns:
            None.

        """
        self.config = config

    def perform_check(self, root, xml_file: str) -> ValidationResult:
        """Perform the validation step for a package.xml file.

        Args:
            root: XML root element.
            xml_file: Path to the XML file.

        Returns:
            ValidationResult containing validity, changes, and messages.

        """
        raise NotImplementedError


class FormatterValidationStep(ValidationStep):
    name = "Formatter validation"

    def __init__(self, config: ValidationConfig, formatter):
        """Initialize formatting validation step.

        Args:
            config: ValidationConfig with feature flags.
            formatter: PackageXmlFormatter instance.

        Returns:
            None.

        """
        super().__init__(config)
        self.formatter = formatter

    def perform_check(self, root, xml_file: str) -> ValidationResult:
        """Run formatting-related checks on a package.xml file.

        Args:
            root: XML root element.
            xml_file: Path to the XML file.

        Returns:
            ValidationResult for formatting checks.

        """
        result = ValidationResult(root=root)
        if self.config.missing_deps_only:
            return result

        if not self.formatter.check_for_non_existing_tags(root, xml_file):
            message = f"Unknown tags found in {xml_file}."
            result.errors.append(message)
            result.critical_errors.append(message)
            result.valid = False

        if self.config.ignore_formatting_errors:
            return result

        checks: list[tuple[str, Callable[..., bool], tuple]] = [
            (
                "Check for empty lines",
                self.formatter.check_for_empty_lines,
                (root, xml_file),
            ),
            (
                "Check for duplicate elements",
                self.formatter.check_for_duplicates,
                (root, xml_file),
            ),
            (
                "Check element occurrences",
                self.formatter.check_element_occurrences,
                (root, xml_file),
            ),
            (
                "Check element order",
                self.formatter.check_element_order,
                (root, xml_file),
            ),
            (
                "Check dependency order",
                self.formatter.check_dependency_order,
                (root, xml_file),
            ),
            (
                "Check indentation",
                self.formatter.check_indentation,
                (root,),
            ),
        ]

        for check_name, check_fn, args in checks:
            ok = check_fn(*args)
            if ok:
                continue
            if self.config.check_only:
                result.errors.append(f"{check_name} failed in {xml_file}.")
                result.valid = False
            else:
                result.warnings.append(f"{check_name} corrected in {xml_file}.")
                result.changed = True
                result.valid = False

        return result


class BuildToolDependStep(ValidationStep):
    name = "Build tool dependency"

    def __init__(self, config: ValidationConfig, formatter):
        """Initialize buildtool dependency validation step.

        Args:
            config: ValidationConfig with feature flags.
            formatter: PackageXmlFormatter instance.

        Returns:
            None.

        """
        super().__init__(config)
        self.formatter = formatter

    def perform_check(self, root, xml_file: str) -> ValidationResult:
        """Validate and optionally fix buildtool dependency tags.

        Args:
            root: XML root element.
            xml_file: Path to the XML file.

        Returns:
            ValidationResult for buildtool dependency checks.

        """
        result = ValidationResult(root=root)
        if self.config.missing_deps_only:
            return result

        pkg_type, is_msg_pkg = get_package_type(xml_file)
        buildtool = [tool.text for tool in root.findall("buildtool_depend")]
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
            return result

        pkg_name = os.path.basename(os.path.dirname(xml_file))
        corrected_buildtool_str = f"<buildtool_depend>{'ament_cmake' if pkg_type == PackageType.CMAKE_PKG else 'ament_python'}</buildtool_depend>"
        if is_msg_pkg:
            corrected_buildtool_str += (
                f" <buildtool_depend>{PackageType.MSG_PKG.value}</buildtool_depend>"
            )

        if self.config.check_only:
            if len(buildtool) == 0:
                result.errors.append(
                    f"Missing <buildtool_depend> tag in {pkg_name}/package.xml. Please include {corrected_buildtool_str}."
                )
            else:
                result.errors.append(
                    f"Incorrect <buildtool_depend> in {pkg_name}/package.xml. Expected {corrected_buildtool_str}, found {buildtool if buildtool is not None else 'None'}."
                )
            result.valid = False
            return result

        if not self.config.auto_fill_missing_deps:
            result.critical_errors.append(
                f"Cannot auto-fill missing <buildtool_depend> in {pkg_name}/package.xml. Please add it manually."
            )
            result.valid = False
            return result

        self.formatter.add_buildtool_depends(
            root,
            [pkg_type.value, PackageType.MSG_PKG.value]
            if is_msg_pkg
            else [pkg_type.value],
        )
        result.warnings.append(
            f"Auto-filling {corrected_buildtool_str} in {pkg_name}/package.xml."
        )
        result.changed = True
        result.valid = False
        return result


class MemberOfGroupStep(ValidationStep):
    name = "Member of group"

    def __init__(self, config: ValidationConfig, formatter):
        """Initialize member_of_group validation step.

        Args:
            config: ValidationConfig with feature flags.
            formatter: PackageXmlFormatter instance.

        Returns:
            None.

        """
        super().__init__(config)
        self.formatter = formatter

    def perform_check(self, root, xml_file: str) -> ValidationResult:
        """Validate and optionally fix member_of_group tag for msg packages.

        Args:
            root: XML root element.
            xml_file: Path to the XML file.

        Returns:
            ValidationResult for member_of_group checks.

        """
        result = ValidationResult(root=root)
        if self.config.missing_deps_only:
            return result

        member_of_group = root.find("member_of_group")
        _, is_msg_pkg = get_package_type(xml_file)
        if not is_msg_pkg:
            return result

        if (
            member_of_group is not None
            and member_of_group.text == "rosidl_interface_packages"
        ):
            return result

        pkg_name = os.path.basename(os.path.dirname(xml_file))
        if self.config.check_only:
            result.errors.append(
                f"Missing or incorrect <member_of_group> in {pkg_name}/package.xml."
            )
            result.valid = False
            return result

        if not self.config.auto_fill_missing_deps:
            result.critical_errors.append(
                f"Cannot auto-fill missing <member_of_group> in {pkg_name}/package.xml. Please add it manually."
            )
            result.valid = False
            return result

        self.formatter.add_member_of_group(root, "rosidl_interface_packages")
        result.warnings.append(
            f"Auto-filling <member_of_group>rosidl_interface_packages</member_of_group> in {pkg_name}/package.xml."
        )
        result.changed = True
        result.valid = False
        return result


class BuildTypeExportStep(ValidationStep):
    name = "Build type export"

    def __init__(self, config: ValidationConfig, formatter):
        """Initialize build type export validation step.

        Args:
            config: ValidationConfig with feature flags.
            formatter: PackageXmlFormatter instance.

        Returns:
            None.

        """
        super().__init__(config)
        self.formatter = formatter

    def perform_check(self, root, xml_file: str) -> ValidationResult:
        """Validate and optionally fix <export><build_type> for packages.

        Args:
            root: XML root element.
            xml_file: Path to the XML file.

        Returns:
            ValidationResult for build type export checks.

        """
        result = ValidationResult(root=root)
        if self.config.missing_deps_only:
            return result

        pkg_type, _ = get_package_type(xml_file)
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
            return result

        pkg_name = os.path.basename(os.path.dirname(xml_file))
        if self.config.check_only:
            if not export_exists:
                result.errors.append(f"Missing <export> tag in {pkg_name}/package.xml.")
            else:
                result.errors.append(
                    f"Incorrect <build_type> in <export> tag in {pkg_name}/package.xml."
                )
            result.valid = False
            return result

        if not self.config.auto_fill_missing_deps:
            result.critical_errors.append(
                f"Cannot auto-fill <export><build_type> in {pkg_name}/package.xml. Please add it manually."
            )
            result.valid = False
            return result

        self.formatter.add_build_type_export(
            root,
            "ament_cmake" if pkg_type == PackageType.CMAKE_PKG else "ament_python",
        )
        result.warnings.append(
            f"Auto-filling <export><build_type>{'ament_cmake' if pkg_type == PackageType.CMAKE_PKG else 'ament_python'}</build_type></export> in {pkg_name}/package.xml."
        )
        result.changed = True
        result.valid = False
        return result


class RosdepCheckStep(ValidationStep):
    name = "ROS dependency check"

    def __init__(self, config: ValidationConfig, formatter, rosdep_validator):
        """Initialize ROS dependency validation step.

        Args:
            config: ValidationConfig with feature flags.
            formatter: PackageXmlFormatter instance.
            rosdep_validator: RosdepValidator instance.

        Returns:
            None.

        """
        super().__init__(config)
        self.formatter = formatter
        self.rosdep_validator = rosdep_validator

    def perform_check(self, root, xml_file: str) -> ValidationResult:
        """Validate rosdep keys in package.xml dependencies.

        Args:
            root: XML root element.
            xml_file: Path to the XML file.

        Returns:
            ValidationResult for rosdep checks.

        """
        result = ValidationResult(root=root)
        if not self.config.check_rosdeps or self.config.missing_deps_only:
            return result

        rosdeps = self.formatter.retrieve_all_dependencies(root)
        if not rosdeps:
            return result

        unresolvable = self.rosdep_validator.check_rosdeps_and_local_pkgs(rosdeps)
        if not unresolvable:
            return result

        pkg_name = os.path.basename(os.path.dirname(xml_file))
        message = (
            f"Unresolvable ROS dependencies found in {pkg_name}/package.xml: "
            f"{', '.join(unresolvable)}"
        )
        result.errors.append(message)
        result.critical_errors.append(message)
        result.valid = False
        return result


class CMakeComparisonStep(ValidationStep):
    name = "CMake dependency comparison"

    def __init__(self, config: ValidationConfig, formatter, rosdep_validator):
        """Initialize CMake comparison validation step.

        Args:
            config: ValidationConfig with feature flags.
            formatter: PackageXmlFormatter instance.
            rosdep_validator: RosdepValidator instance.

        Returns:
            None.

        """
        super().__init__(config)
        self.formatter = formatter
        self.rosdep_validator = rosdep_validator

    def perform_check(self, root, xml_file: str) -> ValidationResult:
        """Compare CMakeLists.txt dependencies to package.xml entries.

        Args:
            root: XML root element.
            xml_file: Path to the XML file.

        Returns:
            ValidationResult for CMake comparison checks.

        """
        result = ValidationResult(root=root)
        if not self.config.compare_with_cmake:
            return result

        pkg_type, _ = get_package_type(xml_file)
        if pkg_type != PackageType.CMAKE_PKG:
            return result

        cmake_file = os.path.join(os.path.dirname(xml_file), "CMakeLists.txt")
        if not os.path.exists(cmake_file):
            message = (
                f"Cannot check for CMake dependencies, {cmake_file} does not exist."
            )
            result.errors.append(message)
            if not self.config.auto_fill_missing_deps:
                result.critical_errors.append(message)
            result.valid = False
            return result

        pkg_name = os.path.basename(os.path.dirname(xml_file))
        build_deps_cmake, test_deps_cmake = read_deps_from_cmake_file(cmake_file)
        build_deps_cmake = [dep for dep in build_deps_cmake if dep != "ament_cmake"]
        test_deps_cmake = [dep for dep in test_deps_cmake if dep != "ament_cmake"]

        build_deps = self.formatter.retrieve_build_dependencies(root)
        test_deps = self.formatter.retrieve_test_dependencies(root)

        def dedupe_dependencies(dependencies: list[str]) -> list[str]:
            """Remove duplicate dependency names while preserving order.

            Args:
                dependencies: List of dependency names.

            Returns:
                Deduplicated list of dependency names.

            """
            seen = set()
            deduped = []
            for dep in dependencies:
                if dep in seen:
                    continue
                seen.add(dep)
                deduped.append(dep)
            return deduped

        def check_dependency_group(
            cmake_deps: list[str],
            xml_deps: list[str],
            dependency_label: str,
            dependency_tag: str,
        ) -> None:
            """Compare CMake dependencies with XML dependencies.

            Args:
                cmake_deps: Dependencies extracted from CMakeLists.txt.
                xml_deps: Dependencies present in package.xml.
                dependency_label: Human-readable label for logging.
                dependency_tag: XML tag name to add when auto-filling.

            Returns:
                None.

            """
            missing_deps = []
            unresolved_deps = []

            for dep in dedupe_dependencies(cmake_deps):
                resolved = self.rosdep_validator.resolve_cmake_dependency(dep)
                if resolved:
                    if resolved not in xml_deps:
                        missing_deps.append(resolved)
                    continue

                candidates = self.rosdep_validator.search_rosdep_candidates(dep)
                if candidates and any(
                    candidate in xml_deps for candidate in candidates
                ):
                    continue
                unresolved_deps.append((dep, candidates))

            if missing_deps:
                deps = "\n\t\t" + "\n\t\t".join(dedupe_dependencies(missing_deps))
                if self.config.check_only or not self.config.auto_fill_missing_deps:
                    message = (
                        f"Missing {dependency_label} in {pkg_name}/package.xml compared to "
                        f"{pkg_name}/CMakeList.txt:{deps}"
                    )
                    result.errors.append(message)
                    result.valid = False
                    if not self.config.auto_fill_missing_deps:
                        result.critical_errors.append(message)
                else:
                    self.formatter.add_dependencies(root, missing_deps, dependency_tag)
                    result.warnings.append(
                        f"Auto-filling missing {dependency_label} in {pkg_name}/package.xml:{deps}"
                    )
                    result.changed = True
                    result.valid = False

            for dep, candidates in unresolved_deps:
                if candidates:
                    candidate_list = ", ".join(candidates)
                    message = (
                        f"Unable to resolve {dependency_label} '{dep}' via rosdep resolve or mapping. "
                        f"rosdep search candidates: {candidate_list}"
                    )
                else:
                    message = f"Unable to resolve {dependency_label} '{dep}' via rosdep resolve, mapping, or rosdep search."
                if self.config.strict_cmake_checking:
                    result.errors.append(message)
                    result.valid = False
                    result.critical_errors.append(message)
                else:
                    result.warnings.append(message)

        check_dependency_group(build_deps_cmake, build_deps, "dependencies", "depend")
        check_dependency_group(
            test_deps_cmake, test_deps, "test dependencies", "test_depend"
        )
        return result


class LaunchDependencyStep(ValidationStep):
    name = "Launch dependency check"

    def __init__(
        self,
        config: ValidationConfig,
        formatter,
        rosdep_validator,
        package_name: str,
        exec_deps: list[str],
        test_deps: list[str],
    ):
        """Initialize launch dependency validation step.

        Args:
            config: ValidationConfig with feature flags.
            formatter: PackageXmlFormatter instance.
            rosdep_validator: RosdepValidator instance or None.
            package_name: Name of the package being checked.
            exec_deps: Existing exec dependencies from package.xml.
            test_deps: Existing test dependencies from package.xml.

        Returns:
            None.

        """
        super().__init__(config)
        self.formatter = formatter
        self.rosdep_validator = rosdep_validator
        self.package_name = package_name
        self.exec_deps = exec_deps
        self.test_deps = test_deps

    def perform_check(self, root, xml_file: str) -> ValidationResult:
        """Validate launch/test file dependencies against package.xml.

        Args:
            root: XML root element.
            xml_file: Path to the XML file.

        Returns:
            ValidationResult for launch dependency checks.

        """
        result = ValidationResult(root=root)

        def extract_launch_deps(folder_names: list[str]) -> list[str]:
            """Extract launch dependencies from listed folders.

            Args:
                folder_names: Folder names to scan relative to package root.

            Returns:
                List of discovered package names.

            """
            launch_deps = []
            for folder in folder_names:
                launch_dir = os.path.join(os.path.dirname(xml_file), folder)
                if os.path.isdir(launch_dir):
                    launch_deps.extend(scan_files(launch_dir))
            return launch_deps

        def validate_launch_folders(
            launch_folder_names: list[str], xml_deps: list[str], depend_tag: str
        ) -> None:
            """Validate and optionally fix dependencies for launch/test folders.

            Args:
                launch_folder_names: Folder names to scan relative to package root.
                xml_deps: Current dependencies from package.xml.
                depend_tag: Dependency tag to add (exec_depend/test_depend).

            Returns:
                None.

            """
            launch_deps = extract_launch_deps(launch_folder_names)
            if not launch_deps:
                return

            missing_deps = [
                dep
                for dep in launch_deps
                if dep not in xml_deps and dep != self.package_name
            ]
            if not missing_deps:
                return

            if self.config.check_only:
                result.errors.append(
                    f"Missing <{depend_tag}> dependencies in {self.package_name}/package.xml: "
                    f"{', '.join(missing_deps)}"
                )
                result.valid = False
                return

            if not self.config.auto_fill_missing_deps:
                result.critical_errors.append(
                    f"Cannot auto-fill missing <{depend_tag}> dependencies: {missing_deps} "
                    f"in {self.package_name}/package.xml. Please add them manually."
                )
                result.valid = False
                return

            if self.config.check_rosdeps:
                invalid_deps = self.rosdep_validator.check_rosdeps_and_local_pkgs(
                    missing_deps
                )
                if invalid_deps:
                    result.critical_errors.append(
                        f"Cannot auto-fill invalid launch dependencies: {', '.join(invalid_deps)}"
                    )
                    result.valid = False
                    return
                missing_deps = [d for d in missing_deps if d not in invalid_deps]

            self.formatter.add_dependencies(root, missing_deps, depend_tag)
            result.warnings.append(
                f"Auto-filling {len(missing_deps)} missing <{depend_tag}> dependencies in "
                f"{self.package_name}/package.xml."
            )
            result.changed = True
            result.valid = False

        validate_launch_folders(["launch", "components"], self.exec_deps, "exec_depend")
        validate_launch_folders(["test"], self.test_deps, "test_depend")
        return result
