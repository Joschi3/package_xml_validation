"""Member-of-group validation step."""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from ..condition_eval import evaluate_condition
from ..logger import get_logger
from ..package_types import get_package_type
from ._base import ValidationConfig, ValidationResult, ValidationStep

if TYPE_CHECKING:
    from ..package_types import XmlElement
    from ..pkg_xml_formatter import PackageXmlFormatter


class MemberOfGroupStep(ValidationStep):
    """Ensure message packages declare ``<member_of_group>rosidl_interface_packages</member_of_group>``.

    Only message packages (those whose ``CMakeLists.txt`` calls
    ``rosidl_generate_interfaces``) are checked; other package types are
    no-ops. Honours REP-149 ``condition="…"`` attributes — a membership
    whose condition evaluates to False is treated as absent, matching
    the behaviour of the other condition-aware steps. Mutates only when
    ``auto_fill_missing_deps=True``; otherwise emits a critical error.
    Skipped when ``missing_deps_only=True``.
    """

    name = "Member of group"

    def __init__(
        self, config: ValidationConfig, formatter: PackageXmlFormatter
    ) -> None:
        """Initialize member_of_group validation step."""
        super().__init__(config)
        self.formatter = formatter
        self._logger = get_logger(__name__)

    def perform_check(self, root: XmlElement, xml_file: str) -> ValidationResult:
        """Validate and optionally fix member_of_group tag for msg packages."""
        result = ValidationResult(root=root)
        if self.config.missing_deps_only:
            return result

        _, is_msg_pkg = get_package_type(xml_file)
        if not is_msg_pkg:
            return result

        if any(
            (m.text or "").strip() == "rosidl_interface_packages"
            and (
                not self.config.evaluate_conditions
                or evaluate_condition(m.get("condition"), logger=self._logger)
            )
            for m in root.findall("member_of_group")
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
