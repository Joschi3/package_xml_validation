"""rosidl_default_runtime exec_depend validation step."""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

from ..condition_eval import evaluate_condition
from ..formatter.dependency_queries import (
    retrieve_exec_dependencies_with_conditions,
)
from ..logger import get_logger
from ..package_types import get_package_type
from ._base import ValidationConfig, ValidationResult, ValidationStep

if TYPE_CHECKING:
    from ..package_types import XmlElement
    from ..pkg_xml_formatter import PackageXmlFormatter


_RUNTIME_DEP = "rosidl_default_runtime"


class RosidlInterfaceRuntimeStep(ValidationStep):
    """Ensure interface (msg/srv/action) packages declare a runtime dependency.

    The ROS 2 rolling "Custom interfaces" tutorial requires interface
    packages to declare ``<exec_depend>rosidl_default_runtime</exec_depend>``;
    without it, generated Python/C++ runtime code is not available to
    consumers at runtime.

    Detection of an interface package piggybacks on
    :func:`get_package_type` (a CMake package whose ``CMakeLists.txt``
    contains ``rosidl_generate_interfaces(…)``). Honours REP-149
    ``condition="…"`` attributes — an inactive ``rosidl_default_runtime``
    does not satisfy the rule.

    Behaviour:
      - check_only or auto_fill_missing_deps=False: report an error.
      - auto_fill_missing_deps=True: insert ``<exec_depend>``.

    Skipped for non-msg packages and when ``missing_deps_only`` is set.
    """

    name = "rosidl_default_runtime check"

    def __init__(
        self, config: ValidationConfig, formatter: PackageXmlFormatter
    ) -> None:
        """Initialize rosidl_default_runtime validation step."""
        super().__init__(config)
        self.formatter = formatter
        self._logger = get_logger(__name__)

    def perform_check(self, root: XmlElement, xml_file: str) -> ValidationResult:
        """Check for an active rosidl_default_runtime exec_depend."""
        result = ValidationResult(root=root)
        if self.config.missing_deps_only:
            return result

        _, is_msg_pkg = get_package_type(xml_file)
        if not is_msg_pkg:
            return result

        if self._has_active_runtime_exec_depend(root):
            return result

        pkg_name = os.path.basename(os.path.dirname(xml_file))
        location = f"{pkg_name}/package.xml"

        if self.config.check_only:
            result.errors.append(
                f"Missing <exec_depend>{_RUNTIME_DEP}</exec_depend> in {location}. "
                "Interface packages need this for runtime code generation."
            )
            result.valid = False
            return result

        if not self.config.auto_fill_missing_deps:
            result.critical_errors.append(
                f"Cannot auto-fill missing <exec_depend>{_RUNTIME_DEP}</exec_depend> "
                f"in {location}. Please add it manually."
            )
            result.valid = False
            return result

        self.formatter.add_dependencies(root, [_RUNTIME_DEP], "exec_depend")
        result.warnings.append(
            f"Auto-filling <exec_depend>{_RUNTIME_DEP}</exec_depend> in {location}."
        )
        result.changed = True
        result.valid = False
        return result

    def _has_active_runtime_exec_depend(self, root: XmlElement) -> bool:
        """True iff an active <exec_depend>rosidl_default_runtime</exec_depend>
        (or equivalent <depend>) is present.

        Both ``<exec_depend>`` and the unified ``<depend>`` cover the exec
        phase per REP-149, so either satisfies the rule.
        """
        for text, cond in retrieve_exec_dependencies_with_conditions(root):
            if text != _RUNTIME_DEP:
                continue
            if not self.config.evaluate_conditions or evaluate_condition(
                cond, logger=self._logger
            ):
                return True
        return False
