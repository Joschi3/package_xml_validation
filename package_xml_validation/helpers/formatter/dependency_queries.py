"""Read-only queries against a parsed package.xml tree."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..package_types import XmlElement


_BUILD_DEPEND_TAGS = frozenset(
    {
        "buildtool_depend",
        "buildtool_export_depend",
        "build_depend",
        "build_export_depend",
        "depend",
    }
)
_TEST_DEPEND_TAGS = frozenset({"test_depend", "depend"})
_EXEC_DEPEND_TAGS = frozenset({"exec_depend", "depend"})


def retrieve_all_dependencies(root: XmlElement) -> list[str]:
    """Return the text of every ``<*depend*>`` child of ``root``."""
    dependencies: list[str] = []
    for elem in root:
        if isinstance(elem.tag, str) and "depend" in elem.tag and elem.text:
            dependencies.append(elem.text.strip())
    return dependencies


def retrieve_build_dependencies(root: XmlElement) -> list[str]:
    """Return build-related dependencies declared in ``root``."""
    return _collect_by_tag(root, _BUILD_DEPEND_TAGS)


def retrieve_test_dependencies(root: XmlElement) -> list[str]:
    """Return test dependencies declared in ``root``."""
    return _collect_by_tag(root, _TEST_DEPEND_TAGS)


def retrieve_exec_dependencies(root: XmlElement) -> list[str]:
    """Return exec dependencies declared in ``root``."""
    return _collect_by_tag(root, _EXEC_DEPEND_TAGS)


def get_package_name(root: XmlElement) -> str | None:
    """Return the package name (text of the ``<name>`` element) or ``None``."""
    name_elem = root.find("name")
    if name_elem is not None and name_elem.text:
        return name_elem.text.strip()
    return None


def _collect_by_tag(root: XmlElement, tags: frozenset[str]) -> list[str]:
    return [
        elem.text.strip()
        for elem in root
        if isinstance(elem.tag, str) and elem.tag in tags and elem.text
    ]
