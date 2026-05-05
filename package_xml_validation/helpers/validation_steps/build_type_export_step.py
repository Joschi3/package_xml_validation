"""Build-type export validation step."""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from ..package_types import PackageType, get_package_type
from ._base import ValidationConfig, ValidationResult, ValidationStep

if TYPE_CHECKING:
    from ..package_types import XmlElement
    from ..pkg_xml_formatter import PackageXmlFormatter


class BuildTypeExportStep(ValidationStep):
    name = "Build type export"

    def __init__(
        self, config: ValidationConfig, formatter: PackageXmlFormatter
    ) -> None:
        """Initialize build type export validation step."""
        super().__init__(config)
        self.formatter = formatter

    def perform_check(self, root: XmlElement, xml_file: str) -> ValidationResult:
        """Validate and optionally fix <export><build_type> for packages."""
        result = ValidationResult(root=root)
        if self.config.missing_deps_only:
            return result

        pkg_type, _ = get_package_type(xml_file)
        export = root.find("export")
        export_exists = export is not None
        build_type = export.find("build_type") if export is not None else None
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
