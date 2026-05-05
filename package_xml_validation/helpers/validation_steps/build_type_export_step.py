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
    """Verify (and optionally insert) ``<export><build_type>`` for build packages.

    Rule: CMake packages must declare ``ament_cmake``; Python packages
    must declare ``ament_python``. Other package types (message-only,
    manifest-only/UNKNOWN) require no ``<build_type>`` and are skipped.

    Reads ``root`` plus the package directory (via ``get_package_type``).
    Mutates only when ``auto_fill_missing_deps=True``; otherwise emits a
    critical error. Skipped when ``missing_deps_only=True``. Other
    ``<export>`` children (besides ``<build_type>``) are left untouched.
    """

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
        expected = _expected_build_type(pkg_type)

        # No expected build_type means the package type doesn't require one;
        # otherwise the existing build_type element must match.
        if expected is None or (build_type is not None and build_type.text == expected):
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

        self.formatter.add_build_type_export(root, expected)
        result.warnings.append(
            f"Auto-filling <export><build_type>{expected}</build_type></export> in {pkg_name}/package.xml."
        )
        result.changed = True
        result.valid = False
        return result


def _expected_build_type(pkg_type: PackageType) -> str | None:
    """Return the required ``<build_type>`` text for ``pkg_type``, or ``None``
    when the package type doesn't require one."""
    if pkg_type == PackageType.CMAKE_PKG:
        return "ament_cmake"
    if pkg_type == PackageType.PYTHON_PKG:
        return "ament_python"
    return None
