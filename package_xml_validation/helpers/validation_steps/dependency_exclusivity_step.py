"""REP-149 ``<depend>`` exclusivity validation step."""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

import lxml.etree as ET

from ._base import ValidationResult, ValidationStep

if TYPE_CHECKING:
    from ..package_types import XmlElement


# Tags that <depend> aggregates per REP-149: "<depend> is shorthand for
# <build_depend>, <build_export_depend>, and <exec_depend>". <test_depend>
# and <buildtool_depend> are NOT covered.
_GRANULAR_TAGS = ("build_depend", "build_export_depend", "exec_depend")


class DependencyExclusivityStep(ValidationStep):
    """Enforce REP-149's ``<depend>`` exclusivity rule.

    REP-149: *"The ``<depend>`` tag cannot be used in combination with
    any of the three equivalent tags for the same package or key
    name."* The three are ``<build_depend>``, ``<build_export_depend>``,
    and ``<exec_depend>``.

    Behaviour:
      - check_only or auto_fill_missing_deps=False: report one error per
        offending key and set valid=False.
      - auto_fill_missing_deps=True: collapse to canonical form by
        removing the granular tag(s) — ``<depend>`` already covers
        build/build_export/exec.

    Auto-fix is suppressed when the granular tag carries a ``condition``
    that the ``<depend>`` does not — the granular tag is then *narrower*,
    not redundant, and dropping it would change semantics.

    Skipped when ``missing_deps_only=True``.
    """

    name = "Dependency exclusivity"

    def perform_check(self, root: XmlElement, xml_file: str) -> ValidationResult:
        """Detect and optionally fix overlapping <depend> entries."""
        result = ValidationResult(root=root)
        if self.config.missing_deps_only:
            return result

        depend_conditions = _depend_conditions_by_key(root)
        if not depend_conditions:
            return result

        offenders = _find_offenders(root, depend_conditions)
        if not offenders:
            return result

        pkg_name = os.path.basename(os.path.dirname(xml_file))
        location = f"{pkg_name}/package.xml"

        if self.config.check_only or not self.config.auto_fill_missing_deps:
            for key, tag, _ in offenders:
                result.errors.append(
                    f"<{tag}>{key}</{tag}> in {location} duplicates the "
                    f"<depend>{key}</depend> entry. REP-149 forbids this; "
                    f"keep only <depend>."
                )
            result.valid = False
            return result

        # Auto-fix: drop redundant granular entries; report the rest.
        for key, tag, elem in offenders:
            if _is_redundant(elem, depend_conditions[key]):
                root.remove(elem)
                result.warnings.append(
                    f"Removed redundant <{tag}>{key}</{tag}> from {location} "
                    f"(already covered by <depend>{key}</depend>)."
                )
                result.changed = True
            else:
                result.errors.append(
                    f"<{tag}>{key}</{tag}> in {location} overlaps with "
                    f"<depend>{key}</depend> but carries a different "
                    f"condition; please resolve manually."
                )
                result.valid = False
        if result.changed:
            result.valid = False
        return result


def _depend_conditions_by_key(root: XmlElement) -> dict[str, set[str | None]]:
    """Map each ``<depend>`` key to the set of ``condition`` values seen on it.

    A key with multiple ``<depend>`` entries carrying different conditions
    is unusual but legal (acts as an OR). We track all of them so the
    redundancy check below can compare exactly.
    """
    out: dict[str, set[str | None]] = {}
    for elem in root.findall("depend"):
        if not elem.text:
            continue
        out.setdefault(elem.text.strip(), set()).add(elem.get("condition"))
    return out


def _find_offenders(
    root: XmlElement,
    depend_conditions: dict[str, set[str | None]],
) -> list[tuple[str, str, XmlElement]]:
    """Return ``(key, tag, elem)`` for every granular tag that shares a
    key with a ``<depend>`` entry."""
    offenders: list[tuple[str, str, XmlElement]] = []
    for elem in root:
        if elem.tag is ET.Comment:  # type: ignore[comparison-overlap]
            continue  # type: ignore[unreachable]
        if not isinstance(elem.tag, str) or elem.tag not in _GRANULAR_TAGS:
            continue
        if not elem.text:
            continue
        key = elem.text.strip()
        if key in depend_conditions:
            offenders.append((key, elem.tag, elem))
    return offenders


def _is_redundant(
    granular_elem: XmlElement, depend_conditions: set[str | None]
) -> bool:
    """A granular tag is safely removable iff its condition matches at
    least one of the conditions on the ``<depend>`` entries for the
    same key. ``None``-vs-``None`` (both unconditional) counts as a match.
    """
    return granular_elem.get("condition") in depend_conditions
