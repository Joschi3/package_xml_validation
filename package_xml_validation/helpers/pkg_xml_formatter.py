from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any
from collections.abc import Iterable

from .formatter import dependency_queries, mutations, structural_checks
from .formatter.constants import ELEMENTS, NEW_LINE, NEW_LINE_BEFORE
from .formatter.indentation import prettyprint as _prettyprint
from .formatter.indentation import resolve_indentation
from .logger import get_logger

if TYPE_CHECKING:
    from .package_types import XmlElement

__all__ = ["ELEMENTS", "NEW_LINE", "NEW_LINE_BEFORE", "PackageXmlFormatter"]


class PackageXmlFormatter:
    def __init__(
        self,
        check_only: bool = False,
        verbose: bool = False,
        logger: logging.Logger | None = None,
    ) -> None:
        """Initialize formatter configuration and logger.

        Args:
            check_only: If True, do not mutate XML when checks fail.
            verbose: If True, enable verbose logging.
            logger: Optional logger instance to use.

        Returns:
            None.

        """
        self.check_only = check_only
        self.logger = (
            logger
            if logger
            else get_logger(__name__, level="verbose" if verbose else "normal")
        )

    def _resolve_indentation(self, root: XmlElement, default: str = "  ") -> str:
        """Infer indentation from the first child tail, or return a default."""
        return resolve_indentation(root, default)

    def prettyprint(self, element: XmlElement, **kwargs: Any) -> None:
        """Print a pretty-printed XML element to stdout."""
        _prettyprint(element, **kwargs)

    def check_dependency_order(self, root: XmlElement, xml_file: str) -> bool:
        """Check and optionally correct dependency ordering."""
        return structural_checks.check_dependency_order(
            root, xml_file, self.check_only, self.logger
        )

    def check_for_duplicates(self, root: XmlElement, xml_file: str) -> bool:
        """Detect (and optionally remove) duplicate elements."""
        return structural_checks.check_for_duplicates(
            root, xml_file, self.check_only, self.logger
        )

    def check_element_occurrences(self, root: XmlElement, xml_file: str) -> bool:
        """Verify element occurrence counts against the schema."""
        return structural_checks.check_element_occurrences(
            root, xml_file, self.check_only, self.logger
        )

    def check_element_order(self, root: XmlElement, xml_file: str) -> bool:
        """Check (and optionally restore) the schema-defined element ordering."""
        return structural_checks.check_element_order(
            root, xml_file, self.check_only, self.logger
        )

    def check_for_empty_lines(self, root: XmlElement, xml_file: str) -> bool:
        """Ensure at most one blank line separates sibling elements."""
        return structural_checks.check_for_empty_lines(
            root, xml_file, self.check_only, self.logger
        )

    def check_indentation(
        self, root: XmlElement, level: int = 1, indentation: str = "  "
    ) -> bool:
        """Recursively validate (and optionally fix) indentation."""
        return structural_checks.check_indentation(
            root, self.check_only, self.logger, level, indentation
        )

    def check_for_non_existing_tags(self, root: XmlElement, xml_file: str) -> bool:
        """Reject tags that are not part of the package.xml schema."""
        return structural_checks.check_for_non_existing_tags(
            root, xml_file, self.logger
        )

    def retrieve_all_dependencies(self, root: XmlElement) -> list[str]:
        """Retrieve all dependency tag values from the XML tree."""
        return dependency_queries.retrieve_all_dependencies(root)

    def retrieve_build_dependencies(self, root: XmlElement) -> list[str]:
        """Retrieve build-related dependencies from the XML tree."""
        return dependency_queries.retrieve_build_dependencies(root)

    def retrieve_test_dependencies(self, root: XmlElement) -> list[str]:
        """Retrieve test dependencies from the XML tree."""
        return dependency_queries.retrieve_test_dependencies(root)

    def retrieve_exec_dependencies(self, root: XmlElement) -> list[str]:
        """Retrieve execution dependencies from the XML tree."""
        return dependency_queries.retrieve_exec_dependencies(root)

    def get_package_name(self, root: XmlElement) -> str | None:
        """Retrieve the package name from the XML tree."""
        return dependency_queries.get_package_name(root)

    def add_dependencies(
        self, root: XmlElement, dependencies: Iterable[str], dep_type: str
    ) -> None:
        """Add dependency elements to the XML tree in sorted order."""
        mutations.add_dependencies(root, dependencies, dep_type)

    def add_build_type_export(self, root: XmlElement, build_type: str) -> None:
        """Add or update the build type export."""
        mutations.add_build_type_export(root, build_type)

    def add_buildtool_depends(self, root: XmlElement, buildtool: list[str]) -> None:
        """Replace the set of buildtool_depend elements."""
        mutations.add_buildtool_depends(root, buildtool)

    def add_member_of_group(self, root: XmlElement, group_name: str) -> None:
        """Insert a member_of_group element before the export block."""
        mutations.add_member_of_group(root, group_name)
