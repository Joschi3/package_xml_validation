"""Shared constants for the package.xml formatter."""

from __future__ import annotations

# (element_name, min_occurrences, max_occurrences); max=None means unbounded.
ELEMENTS: list[tuple[str, int, int | None]] = [
    ("name", 1, 1),
    ("version", 1, 1),
    ("description", 1, 1),
    ("maintainer", 1, None),
    ("license", 1, None),
    ("url", 0, None),
    ("author", 0, None),
    ("buildtool_depend", 0, None),
    ("buildtool_export_depend", 0, None),
    ("build_depend", 0, None),
    ("build_export_depend", 0, None),
    ("depend", 0, None),
    ("exec_depend", 0, None),
    ("doc_depend", 0, None),
    ("test_depend", 0, None),
    ("group_depend", 0, None),
    ("member_of_group", 0, None),
    ("export", 0, 1),
]

NEW_LINE_BEFORE: list[str] = [
    "buildtool_depend",
    "build_depend",
    "depend",
    "exec_depend",
    "doc_depend",
    "test_depend",
    "group_depend",
    "member_of_group",
    "export",
]

NEW_LINE = "\n"
