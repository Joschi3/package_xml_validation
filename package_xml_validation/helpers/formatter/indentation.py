"""Indentation inference and pretty-printing helpers."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import lxml.etree as ET

from .constants import NEW_LINE

if TYPE_CHECKING:
    from ..package_types import XmlElement


def resolve_indentation(root: XmlElement, default: str = "  ") -> str:
    """Infer the indentation string from ``root``'s first child tail.

    Falls back to ``default`` when the tail is missing or empty.
    """
    if len(root) == 0:
        return default
    tail = root[0].tail
    if not tail or not isinstance(tail, str):
        return default
    if NEW_LINE in tail:
        return tail[tail.rfind(NEW_LINE) + 1 :]
    return tail


def prettyprint(element: XmlElement, **kwargs: Any) -> None:
    """Pretty-print ``element`` to stdout via :func:`lxml.etree.tostring`."""
    xml = ET.tostring(element, pretty_print=True, **kwargs)
    print(xml.decode(), end="")
