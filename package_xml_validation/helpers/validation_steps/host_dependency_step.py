"""Per-host dependency validation for launch-manager install sets."""

from __future__ import annotations

from typing import TYPE_CHECKING

from ..launch_manager import HostDependencies, declared_host
from ..logger import get_logger
from ._base import ValidationConfig, ValidationResult, ValidationStep

if TYPE_CHECKING:
    from ..exception_parser import DependencyExceptions
    from ..package_types import XmlElement
    from ..pkg_xml_formatter import PackageXmlFormatter
    from ..rosdep_validator import RosdepValidator


class HostDependencyStep(ValidationStep):
    """Check an install set against what its host actually runs.

    A package claiming a host with ``<export><launch_manager_host>`` is the manifest for
    everything that host launches, even though the launch files live elsewhere. What the host
    runs is derived by following each of its components into the launch tree, so this step
    compares that derivation against what the manifest declares.

    Additive only. The derivation is a lower bound - a controller plugin named by class in a
    parameter file, or a package handed to a generic component as an argument, is named nowhere
    it can reach - so a declared dependency it did not derive is left alone and not reported.
    Removing one would uninstall something the robot needs.

    Nothing is filled from an incomplete derivation. A component that could not be read means
    the host's set is short by an unknown amount, and quietly reporting a shorter list as
    complete is worse than reporting nothing.
    """

    name = "Host dependency check"

    def __init__(
        self,
        config: ValidationConfig,
        formatter: PackageXmlFormatter,
        rosdep_validator: RosdepValidator | None,
        package_name: str | None,
        hosts: HostDependencies,
        exceptions: DependencyExceptions | None = None,
    ) -> None:
        """Initialize the per-host dependency validation step."""
        super().__init__(config)
        self.formatter = formatter
        self.rosdep_validator = rosdep_validator
        self.package_name = package_name
        self.hosts = hosts
        self.exceptions = exceptions
        self._logger = get_logger(__name__)

    def perform_check(self, root: XmlElement, xml_file: str) -> ValidationResult:
        """Compare this install set against the packages its host launches."""
        result = ValidationResult(root=root)
        host = declared_host(root)
        if host is None:
            return result

        if host not in self.hosts.packages:
            known = ", ".join(sorted(self.hosts.packages)) or "none"
            result.errors.append(
                f"{self.package_name}/package.xml claims host '{host}', which no "
                f"launch-manager configuration mentions. Hosts in the configuration: {known}."
            )
            result.valid = False
            return result

        declared = set(self.formatter.retrieve_exec_dependencies(root))
        missing = sorted(
            one
            for one in self.hosts.packages[host]
            if one not in declared
            and one != self.package_name
            and not (self.exceptions and self.exceptions.is_ignored(one))
        )
        if not missing:
            return result

        if not self.hosts.complete:
            result.warnings.append(
                f"Not filling {len(missing)} dependency/ies for host '{host}': "
                f"{self._why_incomplete()}"
            )
            return result

        return self._report_or_fill(result, root, host, missing)

    def _why_incomplete(self) -> str:
        reasons = list(self.hosts.problems)
        if self.hosts.unresolved:
            names = ", ".join(sorted(set(self.hosts.unresolved)))
            reasons.append(f"no definition found for component(s) {names}")
        return "; ".join(reasons)

    def _report_or_fill(
        self,
        result: ValidationResult,
        root: XmlElement,
        host: str,
        missing: list[str],
    ) -> ValidationResult:
        if self.config.check_only or not self.config.auto_fill_missing_deps:
            result.errors.append(
                f"Host '{host}' launches {len(missing)} package(s) that "
                f"{self.package_name}/package.xml does not declare: {', '.join(missing)}"
            )
            result.valid = False
            return result

        if self.config.check_rosdeps and self.rosdep_validator is not None:
            invalid = self.rosdep_validator.check_rosdeps_and_local_pkgs(missing)
            if invalid:
                result.critical_errors.append(
                    f"Cannot auto-fill host dependencies that resolve to nothing: "
                    f"{', '.join(sorted(invalid))}"
                )
                result.valid = False
                return result

        self.formatter.add_dependencies(root, missing, "exec_depend")
        result.warnings.append(
            f"Auto-filling {len(missing)} <exec_depend> entries for host '{host}' in "
            f"{self.package_name}/package.xml: {', '.join(missing)}"
        )
        result.changed = True
        result.valid = False
        return result
