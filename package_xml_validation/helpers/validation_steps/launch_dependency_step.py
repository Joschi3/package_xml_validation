"""Launch-folder dependency validation step."""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from ..condition_eval import evaluate_condition
from ..exception_parser import DependencyExceptions
from ..find_launch_dependencies import scan_files
from ..formatter.dependency_queries import (
    retrieve_exec_dependencies_with_conditions,
    retrieve_test_dependencies_with_conditions,
)
from ..logger import get_logger
from ._base import ValidationConfig, ValidationResult, ValidationStep

if TYPE_CHECKING:
    from ..package_types import XmlElement
    from ..pkg_xml_formatter import PackageXmlFormatter
    from ..rosdep_validator import RosdepValidator


class LaunchDependencyStep(ValidationStep):
    """Cross-check packages referenced from launch files against package.xml.

    Scans ``launch/`` and ``components/`` folders for package references
    (via ``find_launch_dependencies.scan_files``); anything not already
    in ``<exec_depend>`` is reported. The ``test/`` folder is checked
    against ``<test_depend>``. The package's own name is excluded.

    Honours REP-149 ``condition="…"`` attributes: an inactive manifest
    dependency does not satisfy a launch reference, since the dep won't
    be installed at runtime in environments where its condition is
    False. Disable with ``--ignore-conditions``.

    When ``auto_fill_missing_deps=True``, missing deps are validated
    against rosdep before being added — names that don't resolve are
    flagged rather than auto-filled. When ``check_only=True`` or
    auto-fill is off, missing deps become errors.
    """

    name = "Launch dependency check"

    def __init__(
        self,
        config: ValidationConfig,
        formatter: PackageXmlFormatter,
        rosdep_validator: RosdepValidator | None,
        package_name: str | None,
        exceptions: DependencyExceptions | None = None,
    ) -> None:
        """Initialize launch dependency validation step."""
        super().__init__(config)
        self.formatter = formatter
        self.rosdep_validator = rosdep_validator
        self.package_name = package_name
        self.exceptions = exceptions or DependencyExceptions()
        self._logger = get_logger(__name__)

    def perform_check(self, root: XmlElement, xml_file: str) -> ValidationResult:
        """Validate launch/test file dependencies against package.xml."""
        result = ValidationResult(root=root)
        if self.config.skip_launch_dep_check:
            return result
        exec_deps = self._active_dep_names(
            retrieve_exec_dependencies_with_conditions(root)
        )
        test_deps = self._active_dep_names(
            retrieve_test_dependencies_with_conditions(root)
        )
        self._validate_folders(
            result, root, xml_file, ["launch", "components"], exec_deps, "exec_depend"
        )
        self._validate_folders(
            result, root, xml_file, ["test"], test_deps, "test_depend"
        )
        return result

    def _active_dep_names(
        self, deps_with_conditions: list[tuple[str, str | None]]
    ) -> list[str]:
        """Filter ``(text, condition)`` pairs by active condition."""
        if not self.config.evaluate_conditions:
            return [text for text, _ in deps_with_conditions]
        return [
            text
            for text, cond in deps_with_conditions
            if evaluate_condition(cond, logger=self._logger)
        ]

    def _validate_folders(
        self,
        result: ValidationResult,
        root: XmlElement,
        xml_file: str,
        launch_folder_names: list[str],
        xml_deps: list[str],
        depend_tag: str,
    ) -> None:
        """Validate (and optionally auto-fill) deps for one folder group."""
        launch_deps = self._extract_launch_deps(xml_file, launch_folder_names)
        if not launch_deps:
            return

        missing_deps = [
            dep
            for dep in launch_deps
            if dep not in xml_deps
            and dep != self.package_name
            and not self.exceptions.is_ignored(dep)
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

        if self.config.check_rosdeps and self.rosdep_validator is not None:
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

    @staticmethod
    def _extract_launch_deps(xml_file: str, folder_names: list[str]) -> list[str]:
        launch_deps: list[str] = []
        for folder in folder_names:
            launch_dir = os.path.join(os.path.dirname(xml_file), folder)
            if os.path.isdir(launch_dir):
                launch_deps.extend(scan_files(launch_dir))
        return launch_deps
