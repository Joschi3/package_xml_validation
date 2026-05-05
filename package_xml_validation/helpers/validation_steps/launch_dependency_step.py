"""Launch-folder dependency validation step."""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from ..find_launch_dependencies import scan_files
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
        exec_deps: list[str],
        test_deps: list[str],
    ) -> None:
        """Initialize launch dependency validation step."""
        super().__init__(config)
        self.formatter = formatter
        self.rosdep_validator = rosdep_validator
        self.package_name = package_name
        self.exec_deps = exec_deps
        self.test_deps = test_deps

    def perform_check(self, root: XmlElement, xml_file: str) -> ValidationResult:
        """Validate launch/test file dependencies against package.xml."""
        result = ValidationResult(root=root)
        self._validate_folders(
            result,
            root,
            xml_file,
            ["launch", "components"],
            self.exec_deps,
            "exec_depend",
        )
        self._validate_folders(
            result, root, xml_file, ["test"], self.test_deps, "test_depend"
        )
        return result

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
