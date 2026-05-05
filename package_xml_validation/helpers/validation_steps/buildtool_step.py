"""Buildtool dependency validation step."""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from ..package_types import PackageType, get_package_type
from ._base import ValidationConfig, ValidationResult, ValidationStep

if TYPE_CHECKING:
    from ..package_types import XmlElement
    from ..pkg_xml_formatter import PackageXmlFormatter


class BuildToolDependStep(ValidationStep):
    name = "Build tool dependency"

    def __init__(
        self, config: ValidationConfig, formatter: PackageXmlFormatter
    ) -> None:
        """Initialize buildtool dependency validation step."""
        super().__init__(config)
        self.formatter = formatter

    def perform_check(self, root: XmlElement, xml_file: str) -> ValidationResult:
        """Validate and optionally fix buildtool dependency tags."""
        result = ValidationResult(root=root)
        if self.config.missing_deps_only:
            return result

        pkg_type, is_msg_pkg = get_package_type(xml_file)
        # Manifest-only package (no CMakeLists.txt, no setup.py): we have no
        # signal to pick a buildtool. Skip the check rather than silently
        # auto-filling ament_cmake.
        if pkg_type == PackageType.UNKNOWN:
            pkg_name = os.path.basename(os.path.dirname(xml_file))
            result.warnings.append(
                f"Skipping buildtool check for {pkg_name}/package.xml: "
                "neither CMakeLists.txt nor setup.py present."
            )
            return result

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
