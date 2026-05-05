"""Parse validator exception directives from XML comments in package.xml."""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import lxml.etree as ET

if TYPE_CHECKING:
    from .package_types import XmlElement

# Regex pattern for parsing validator:ignore directives
_IGNORE_DEPS_PATTERN = re.compile(r"validator:ignore\s+(.+)")


@dataclass(frozen=True)
class DependencyExceptions:
    """Holds per-package dependency exceptions parsed from XML comments."""

    ignored_deps: frozenset[str] = field(default_factory=frozenset)

    def is_ignored(self, dep_name: str) -> bool:
        """Check whether a dependency should be excluded from validation.

        Args:
            dep_name: The dependency name (e.g., 'rclpy').

        Returns:
            True if the dependency should be skipped.

        """
        return dep_name in self.ignored_deps


def parse_exceptions(root: XmlElement) -> DependencyExceptions:
    """Parse validator:ignore directives from XML comments in package.xml.

    Scans all direct children of the root element for comments matching
    the pattern ``<!-- validator:ignore dep1 dep2 ... -->``. Multiple
    such comments are allowed and their dependency lists are merged.

    Args:
        root: lxml root element of package.xml.

    Returns:
        DependencyExceptions containing all parsed exceptions.

    """
    ignored_deps: set[str] = set()

    for node in root:
        if not callable(node.tag):
            continue
        if node.tag is not ET.Comment:
            continue

        text = node.text.strip() if node.text else ""
        match = _IGNORE_DEPS_PATTERN.match(text)
        if match:
            deps = match.group(1).split()
            ignored_deps.update(deps)

    return DependencyExceptions(ignored_deps=frozenset(ignored_deps))
