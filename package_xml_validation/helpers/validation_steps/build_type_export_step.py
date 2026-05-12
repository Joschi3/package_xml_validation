"""Build-type export validation step."""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from ..condition_eval import evaluate_condition
from ..logger import get_logger
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
        self._logger = get_logger(__name__)

    def perform_check(self, root: XmlElement, xml_file: str) -> ValidationResult:
        """Validate and optionally fix <export><build_type> for packages."""
        result = ValidationResult(root=root)
        if self.config.missing_deps_only:
            return result

        pkg_type, _ = get_package_type(xml_file)
        export = root.find("export")
        export_exists = export is not None
        # REP-149: a <build_type> with a condition that evaluates to False
        # doesn't apply, so we should treat it as absent for matching. When
        # multiple are active, REP-149 specifies that the last one wins.
        build_type = (
            self._find_active_build_type(export, result, xml_file)
            if export is not None
            else None
        )
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

    def _find_active_build_type(
        self, export: XmlElement, result: ValidationResult, xml_file: str
    ) -> XmlElement | None:
        """Return the active ``<build_type>``, applying REP-149's last-wins rule.

        REP-149: *"Only one build type should be active after conditions are
        evaluated. If multiple are active then the last build type is to be
        used."* When more than one is active we honour the spec but also
        emit a warning — multiple unconditional ``<build_type>`` entries
        almost always indicate a config mistake.

        With ``evaluate_conditions=False`` every ``<build_type>`` is
        considered active and the last one wins (matches REP-149 fallback).
        """
        actives: list[XmlElement] = [
            bt
            for bt in export.findall("build_type")
            if not self.config.evaluate_conditions
            or evaluate_condition(bt.get("condition"), logger=self._logger)
        ]
        if not actives:
            return None
        if len(actives) > 1:
            pkg_name = os.path.basename(os.path.dirname(xml_file))
            tags = ", ".join(bt.text or "" for bt in actives)
            result.warnings.append(
                f"{pkg_name}/package.xml has multiple active <build_type> "
                f"entries ({tags}); REP-149 says the last one wins."
            )
        return actives[-1]


def _expected_build_type(pkg_type: PackageType) -> str | None:
    """Return the required ``<build_type>`` text for ``pkg_type``, or ``None``
    when the package type doesn't require one."""
    if pkg_type == PackageType.CMAKE_PKG:
        return "ament_cmake"
    if pkg_type == PackageType.PYTHON_PKG:
        return "ament_python"
    return None
