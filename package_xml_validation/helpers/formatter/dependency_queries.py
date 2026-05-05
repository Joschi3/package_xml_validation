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
    return [text for text, _ in retrieve_all_dependencies_with_conditions(root)]


def retrieve_build_dependencies(root: XmlElement) -> list[str]:
    """Return build-related dependencies declared in ``root``."""
    return [text for text, _ in retrieve_build_dependencies_with_conditions(root)]


def retrieve_test_dependencies(root: XmlElement) -> list[str]:
    """Return test dependencies declared in ``root``."""
    return [text for text, _ in retrieve_test_dependencies_with_conditions(root)]


def retrieve_exec_dependencies(root: XmlElement) -> list[str]:
    """Return exec dependencies declared in ``root``."""
    return [text for text, _ in retrieve_exec_dependencies_with_conditions(root)]


def retrieve_all_dependencies_with_conditions(
    root: XmlElement,
) -> list[tuple[str, str | None]]:
    """Like :func:`retrieve_all_dependencies` but also yields each entry's
    REP-149 ``condition="…"`` attribute (``None`` if unset)."""
    return [
        (elem.text.strip(), elem.get("condition"))
        for elem in root
        if isinstance(elem.tag, str) and "depend" in elem.tag and elem.text
    ]


def retrieve_build_dependencies_with_conditions(
    root: XmlElement,
) -> list[tuple[str, str | None]]:
    """Build-related deps with their ``condition`` attribute."""
    return _collect_by_tag_with_conditions(root, _BUILD_DEPEND_TAGS)


def retrieve_test_dependencies_with_conditions(
    root: XmlElement,
) -> list[tuple[str, str | None]]:
    """Test deps with their ``condition`` attribute."""
    return _collect_by_tag_with_conditions(root, _TEST_DEPEND_TAGS)


def retrieve_exec_dependencies_with_conditions(
    root: XmlElement,
) -> list[tuple[str, str | None]]:
    """Exec deps with their ``condition`` attribute."""
    return _collect_by_tag_with_conditions(root, _EXEC_DEPEND_TAGS)


def get_package_name(root: XmlElement) -> str | None:
    """Return the package name (text of the ``<name>`` element) or ``None``."""
    name_elem = root.find("name")
    if name_elem is not None and name_elem.text:
        return name_elem.text.strip()
    return None


def _collect_by_tag_with_conditions(
    root: XmlElement, tags: frozenset[str]
) -> list[tuple[str, str | None]]:
    return [
        (elem.text.strip(), elem.get("condition"))
        for elem in root
        if isinstance(elem.tag, str) and elem.tag in tags and elem.text
    ]
