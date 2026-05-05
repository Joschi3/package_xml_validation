"""Shared base types for validation steps."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..package_types import XmlElement


@dataclass(frozen=True)
class ValidationConfig:
    check_only: bool
    auto_fill_missing_deps: bool
    check_rosdeps: bool
    compare_with_cmake: bool
    strict_cmake_checking: bool
    missing_deps_only: bool
    ignore_formatting_errors: bool
    cmake_keys_no_rosdep: frozenset[str] = frozenset()


@dataclass
class ValidationResult:
    root: XmlElement
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    critical_errors: list[str] = field(default_factory=list)
    changed: bool = False
    valid: bool = True


class ValidationStep:
    name = "Validation step"

    def __init__(self, config: ValidationConfig) -> None:
        """Initialize a validation step with configuration."""
        self.config = config

    def perform_check(self, root: XmlElement, xml_file: str) -> ValidationResult:
        """Perform the validation step for a package.xml file."""
        raise NotImplementedError
