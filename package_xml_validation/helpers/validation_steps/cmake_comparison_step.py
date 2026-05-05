"""CMake-vs-package.xml comparison step."""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from ..cmake_parsers import read_deps_from_cmake_file
from ..package_types import PackageType, get_package_type
from ._base import ValidationConfig, ValidationResult, ValidationStep

if TYPE_CHECKING:
    from ..package_types import XmlElement
    from ..pkg_xml_formatter import PackageXmlFormatter
    from ..rosdep_validator import RosdepValidator


class CMakeComparisonStep(ValidationStep):
    """Cross-check ``find_package(...)`` calls in ``CMakeLists.txt`` against
    ``<build_depend>``/``<test_depend>`` in package.xml.

    For each CMake dependency, attempts (in order) direct rosdep
    resolution, the project's CMake→rosdep mapping, and a fuzzy rosdep
    search. Missing entries are reported, or auto-filled when
    ``auto_fill_missing_deps=True``. Unresolvable entries become
    warnings unless ``strict_cmake_checking=True``, in which case they
    escalate to critical errors.

    Only runs for CMake packages with ``compare_with_cmake=True``;
    Python and manifest-only packages are no-ops.
    """

    name = "CMake dependency comparison"

    def __init__(
        self,
        config: ValidationConfig,
        formatter: PackageXmlFormatter,
        rosdep_validator: RosdepValidator,
    ) -> None:
        """Initialize CMake comparison validation step."""
        super().__init__(config)
        self.formatter = formatter
        self.rosdep_validator = rosdep_validator

    def perform_check(self, root: XmlElement, xml_file: str) -> ValidationResult:
        """Compare CMakeLists.txt dependencies to package.xml entries."""
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
            if not self.config.auto_fill_missing_deps:
                result.critical_errors.append(message)
            else:
                result.errors.append(message)
            result.valid = False
            return result

        pkg_name = os.path.basename(os.path.dirname(xml_file))
        build_deps_cmake, test_deps_cmake = read_deps_from_cmake_file(
            cmake_file, self.config.cmake_keys_no_rosdep or None
        )

        build_deps = self.formatter.retrieve_build_dependencies(root)
        test_deps = self.formatter.retrieve_test_dependencies(root)

        self._compare_group(
            result,
            root,
            pkg_name,
            build_deps_cmake,
            build_deps,
            "dependencies",
            "depend",
        )
        self._compare_group(
            result,
            root,
            pkg_name,
            test_deps_cmake,
            test_deps,
            "test dependencies",
            "test_depend",
        )
        return result

    def _compare_group(
        self,
        result: ValidationResult,
        root: XmlElement,
        pkg_name: str,
        cmake_deps: list[str],
        xml_deps: list[str],
        dependency_label: str,
        dependency_tag: str,
    ) -> None:
        """Compare one CMake dependency group with the matching package.xml tags."""
        missing_deps: list[str] = []
        unresolved_deps: list[tuple[str, list[str]]] = []

        for dep in _dedupe(cmake_deps):
            resolved = self.rosdep_validator.resolve_cmake_dependency(dep)
            if resolved:
                if resolved not in xml_deps:
                    missing_deps.append(resolved)
                continue

            candidates = self.rosdep_validator.search_rosdep_candidates(dep)
            if candidates and any(candidate in xml_deps for candidate in candidates):
                continue
            unresolved_deps.append((dep, candidates))

        if missing_deps:
            deps = "\n\t\t" + "\n\t\t".join(_dedupe(missing_deps))
            if self.config.check_only or not self.config.auto_fill_missing_deps:
                message = (
                    f"Missing {dependency_label} in {pkg_name}/package.xml compared to "
                    f"{pkg_name}/CMakeList.txt:{deps}"
                )
                result.valid = False
                if not self.config.auto_fill_missing_deps:
                    result.critical_errors.append(message)
                else:
                    result.errors.append(message)
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
                result.valid = False
                result.critical_errors.append(message)
            else:
                result.warnings.append(message)


def _dedupe(dependencies: list[str]) -> list[str]:
    seen: set[str] = set()
    deduped: list[str] = []
    for dep in dependencies:
        if dep in seen:
            continue
        seen.add(dep)
        deduped.append(dep)
    return deduped
