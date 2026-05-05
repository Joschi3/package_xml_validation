"""Formatting/structural validation step."""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable

from ._base import ValidationConfig, ValidationResult, ValidationStep

if TYPE_CHECKING:
    from ..package_types import XmlElement
    from ..pkg_xml_formatter import PackageXmlFormatter


class FormatterValidationStep(ValidationStep):
    """Run all structural/formatting checks against the package.xml tree.

    Enforces: schema-allowlisted tag set, element ordering, occurrence
    bounds, dependency ordering, blank-line policy, indentation, no
    duplicate elements.

    Reads: ``root`` only. Mutates the tree to fix any check that fails
    when ``check_only=False``. Skipped entirely when
    ``missing_deps_only=True`` and partially when
    ``ignore_formatting_errors=True`` (only the unknown-tag check still
    runs in that mode).
    """

    name = "Formatter validation"

    def __init__(
        self, config: ValidationConfig, formatter: PackageXmlFormatter
    ) -> None:
        """Initialize formatting validation step."""
        super().__init__(config)
        self.formatter = formatter

    def perform_check(self, root: XmlElement, xml_file: str) -> ValidationResult:
        """Run formatting-related checks on a package.xml file."""
        result = ValidationResult(root=root)
        if self.config.missing_deps_only:
            return result

        if not self.formatter.check_for_non_existing_tags(root, xml_file):
            message = f"Unknown tags found in {xml_file}."
            result.critical_errors.append(message)
            result.valid = False

        if self.config.ignore_formatting_errors:
            return result

        checks: list[tuple[str, Callable[..., bool], tuple]] = [
            (
                "Check for empty lines",
                self.formatter.check_for_empty_lines,
                (root, xml_file),
            ),
            (
                "Check for duplicate elements",
                self.formatter.check_for_duplicates,
                (root, xml_file),
            ),
            (
                "Check element occurrences",
                self.formatter.check_element_occurrences,
                (root, xml_file),
            ),
            (
                "Check element order",
                self.formatter.check_element_order,
                (root, xml_file),
            ),
            (
                "Check dependency order",
                self.formatter.check_dependency_order,
                (root, xml_file),
            ),
            (
                "Check indentation",
                self.formatter.check_indentation,
                (root,),
            ),
        ]

        for check_name, check_fn, args in checks:
            ok = check_fn(*args)
            if ok:
                continue
            if self.config.check_only:
                result.errors.append(f"{check_name} failed in {xml_file}.")
                result.valid = False
            else:
                result.warnings.append(f"{check_name} corrected in {xml_file}.")
                result.changed = True
                result.valid = False

        return result
