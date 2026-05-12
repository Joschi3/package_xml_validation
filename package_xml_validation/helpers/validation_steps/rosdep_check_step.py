"""Rosdep validation step."""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from ..condition_eval import evaluate_condition
from ..formatter.dependency_queries import (
    retrieve_all_dependencies_with_conditions,
)
from ..logger import get_logger
from ._base import ValidationConfig, ValidationResult, ValidationStep

if TYPE_CHECKING:
    from ..package_types import XmlElement
    from ..pkg_xml_formatter import PackageXmlFormatter
    from ..rosdep_validator import RosdepValidator


class RosdepCheckStep(ValidationStep):
    """Verify every ``<*_depend>`` value resolves via rosdep or workspace.

    For each dependency tag value, asks the ``RosdepValidator`` whether it
    resolves either as a rosdep key for the current platform or as a
    local workspace package. Anything else is reported as a critical
    error.

    Honours REP-149 ``condition="…"`` attributes: dependencies whose
    condition evaluates to ``False`` against ``os.environ`` are skipped,
    matching what colcon/rosdep do at build time. Set
    ``evaluate_conditions=False`` (CLI: ``--ignore-conditions``) to fall
    back to evaluating every entry regardless of its condition.

    Read-only — never mutates the tree. Skipped when ``check_rosdeps=False``
    or ``missing_deps_only=True``.
    """

    name = "ROS dependency check"

    def __init__(
        self,
        config: ValidationConfig,
        formatter: PackageXmlFormatter,
        rosdep_validator: RosdepValidator,
    ) -> None:
        """Initialize ROS dependency validation step."""
        super().__init__(config)
        self.formatter = formatter
        self.rosdep_validator = rosdep_validator
        self._logger = get_logger(__name__)

    def perform_check(self, root: XmlElement, xml_file: str) -> ValidationResult:
        """Validate rosdep keys in package.xml dependencies."""
        result = ValidationResult(root=root)
        if not self.config.check_rosdeps or self.config.missing_deps_only:
            return result

        if self.config.evaluate_conditions:
            rosdeps = [
                text
                for text, cond in retrieve_all_dependencies_with_conditions(root)
                if evaluate_condition(cond, logger=self._logger)
            ]
        else:
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
        result.critical_errors.append(message)
        result.valid = False
        return result
