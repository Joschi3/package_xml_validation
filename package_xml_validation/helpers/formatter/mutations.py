"""In-place mutators for a parsed package.xml tree."""

from __future__ import annotations

from collections.abc import Iterable
from typing import TYPE_CHECKING

import lxml.etree as ET

from .constants import ELEMENTS, NEW_LINE
from .indentation import resolve_indentation

if TYPE_CHECKING:
    from ..package_types import XmlElement


def add_dependencies(
    root: XmlElement, dependencies: Iterable[str], dep_type: str
) -> None:
    """Insert ``<dep_type>`` entries into ``root`` in alphabetical order."""
    dep_types = [dep[0] for dep in ELEMENTS if "depend" in dep[0]]
    elements = [dep[0] for dep in ELEMENTS]
    if dep_type not in dep_types:
        raise ValueError(f"Invalid dependency type: {dep_type}")
    indentation = resolve_indentation(root)
    insert_position: int = 0
    first_of_group: int | None = 0
    for dep in dependencies:
        new_elem = ET.Element(dep_type)
        new_elem.text = dep
        new_elem.tail = NEW_LINE + indentation
        # add element to root at correct position -> correct dep group and alphabetical order
        # case 1: dependency group is empty
        if not root.findall(dep_type):
            previous_tag = elements[elements.index(dep_type) - 1]
            while not root.findall(previous_tag):
                previous_tag = elements[elements.index(previous_tag) - 1]
            # find last element with previous_tag
            last_element_count = 0
            for count, elm in enumerate(root):
                if isinstance(elm.tag, str) and elm.tag == previous_tag:
                    last_element_count = count
            insert_position = last_element_count + 1
            first_of_group = insert_position
        # case 2: dependency group is not empty
        else:
            # assume list is sorted -> insert at correct position
            insert_position = 0
            first_of_group = None
            for i, elm in enumerate(root):
                if isinstance(elm.tag, str) and elm.tag == dep_type:
                    if first_of_group is None:
                        first_of_group = i
                        insert_position = i
                    if (elm.text or "") < (new_elem.text or ""):
                        insert_position = i + 1
        root.insert(insert_position, new_elem)
        # adapt empty lines -> in case element prior ends with empty line move it to the new element
        if (
            insert_position > 0
            and first_of_group is not None
            and insert_position > first_of_group
        ):
            prev_elem = root[insert_position - 1]
            if prev_elem.tail and prev_elem.tail.count(NEW_LINE) > 1:
                new_elem.tail = prev_elem.tail
                prev_elem.tail = NEW_LINE + indentation
        if insert_position < len(root) - 1:
            # if next tag is different than the new element, add empty line
            next_element = root[insert_position + 1]
            if next_element.tag != new_elem.tag:
                new_elem.tail = "\n\n" + indentation


def add_build_type_export(root: XmlElement, build_type: str) -> None:
    """Set or insert the ``<export><build_type>`` element.

    Other ``<export>`` children (besides ``<build_type>``) are left untouched.
    """
    indentation = resolve_indentation(root)
    export = root.find("export")
    if export is None:
        export = ET.Element("export")
        # get last element in root
        if len(root) > 0:
            last_element = root[-1]
            if last_element.tail:
                last_element.tail = "\n\n" + indentation
        export.tail = NEW_LINE
        root.append(export)
    build_type_elem = export.find("build_type")
    if build_type_elem is None:
        build_type_elem = ET.Element("build_type")
        build_type_elem.tail = NEW_LINE + indentation
        export.append(build_type_elem)
    build_type_elem.text = build_type
    export.text = NEW_LINE + 2 * indentation


def add_buildtool_depends(root: XmlElement, buildtool: list[str]) -> None:
    """Replace any existing ``<buildtool_depend>`` entries with ``buildtool``."""
    indentation = resolve_indentation(root)
    # 1. clear existing buildtool_depend elements
    for elem in root.findall("buildtool_depend"):
        root.remove(elem)
    # 2. insertposition -> after license, url, or author element
    insert_position = 0
    for i, elem in enumerate(root):
        if isinstance(elem.tag, str) and elem.tag == "license":
            insert_position = i + 1
        elif isinstance(elem.tag, str) and elem.tag == "url":
            insert_position = i + 1
        elif isinstance(elem.tag, str) and elem.tag == "author":
            insert_position = i + 1
    # 3. add buildtool_depend elements
    for i, tool in enumerate(buildtool):
        is_last = i == len(buildtool) - 1
        new_elem = ET.Element("buildtool_depend")
        new_elem.text = tool
        new_elem.tail = "\n\n" if is_last else NEW_LINE
        new_elem.tail += indentation
        root.insert(insert_position, new_elem)
        insert_position += 1


def add_member_of_group(root: XmlElement, group_name: str) -> None:
    """Insert a ``<member_of_group>`` element before ``<export>``."""
    indentation = resolve_indentation(root)
    member_of_group = ET.Element("member_of_group")
    member_of_group.text = group_name
    member_of_group.tail = "\n\n" + indentation
    # insert position -> right before export or at the end
    insert_position = len(root)
    for i, elem in enumerate(root):
        if isinstance(elem.tag, str) and elem.tag == "export":
            insert_position = i
            break
    root.insert(insert_position, member_of_group)
