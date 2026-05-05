"""Structural validators for parsed package.xml trees.

Each ``check_*`` function returns ``True`` when the tree is already valid (or
was corrected without finding anything wrong) and ``False`` when an issue was
found. When ``check_only`` is ``False`` the functions also mutate the tree to
fix what they can.

Some checks additionally expose a ``*_detailed`` variant that returns a
:class:`CheckOutcome`, distinguishing ``FIXED`` (mutated successfully) from
``UNFIXABLE`` (a finding that requires manual intervention). Callers that
need to surface unfixable findings as errors should prefer the detailed
form.
"""

from __future__ import annotations

import logging
from copy import deepcopy
from enum import Enum
from typing import TYPE_CHECKING

import lxml.etree as ET

from .constants import ELEMENTS, NEW_LINE, NEW_LINE_BEFORE
from .indentation import resolve_indentation

if TYPE_CHECKING:
    from ..package_types import XmlElement


class CheckOutcome(Enum):
    """Result of a structural check that can fail in two distinct ways."""

    VALID = "valid"  #: Tree was already valid; nothing to do.
    FIXED = "fixed"  #: Tree had an issue and the check mutated it to fix.
    UNFIXABLE = "unfixable"  #: Tree has an issue that requires manual fixing.


def check_dependency_order(
    root: XmlElement,
    xml_file: str,
    check_only: bool,
    logger: logging.Logger,
) -> bool:
    """Check, and optionally correct, dependency ordering."""
    dependency_order = [elm[0] for elm in ELEMENTS]
    dependencies_with_comments, current_order = _collect_dependencies_with_comments(
        root, dependency_order
    )
    order_mismatch = _detect_dependency_order_mismatch(
        dependencies_with_comments, current_order, dependency_order, xml_file, logger
    )

    if check_only:
        if order_mismatch:
            logger.error(f"Dependency order in {xml_file} is incorrect.")
            return False
        return True

    if not order_mismatch:
        return True

    _reinsert_dependencies_in_order(root, dependencies_with_comments, dependency_order)
    logger.info(f"Corrected dependency order in {xml_file}.")
    return False


_DepsWithComments = dict[str, list[tuple["XmlElement", list["XmlElement"]]]]


def _collect_dependencies_with_comments(
    root: XmlElement, dependency_order: list[str]
) -> tuple[_DepsWithComments, list[str]]:
    """Walk ``root`` once, grouping dependency elements with their preceding comments.

    Returns a ``(dependencies_with_comments, observed_order)`` pair where
    ``observed_order`` is the dep-type sequence as it appears in the tree.
    """
    dependencies_with_comments: _DepsWithComments = {
        dep: [] for dep in dependency_order
    }
    observed_order: list[str] = []
    current_comments: list[XmlElement] = []
    for elem in root:
        # lxml-stubs types ``elem.tag`` as ``str``; at runtime comment
        # nodes have ``tag is ET.Comment`` (a callable), so the identity
        # check is intentional.
        if elem.tag is ET.Comment:  # type: ignore[comparison-overlap]
            current_comments.append(elem)  # type: ignore[unreachable]
        if isinstance(elem.tag, str) and elem.tag in dependency_order:
            dependencies_with_comments[elem.tag].append((elem, current_comments))
            current_comments = []
            observed_order.append(elem.tag)
    return dependencies_with_comments, observed_order


def _detect_dependency_order_mismatch(
    dependencies_with_comments: _DepsWithComments,
    observed_order: list[str],
    dependency_order: list[str],
    xml_file: str,
    logger: logging.Logger,
) -> bool:
    """Return True when the tree violates either type-grouping or alphabetical order."""
    expected_order: list[str] = []
    for dep_type in dependency_order:
        expected_order.extend([dep_type] * len(dependencies_with_comments[dep_type]))

    if observed_order != expected_order:
        logger.error(f"Dependency order in {xml_file} is incorrect.")
        return True

    for dep_type, elem_with_comments in dependencies_with_comments.items():
        names = [e[0].text or "" for e in elem_with_comments]
        if names != sorted(names):
            logger.debug(
                f"Dependency order in {xml_file} is incorrect: {dep_type} elements are not sorted."
            )
            return True

    return False


def _reinsert_dependencies_in_order(
    root: XmlElement,
    dependencies_with_comments: _DepsWithComments,
    dependency_order: list[str],
) -> None:
    """Remove all collected dependency nodes and reinsert them sorted, with
    the schema-defined blank-line policy between groups."""
    indentation = resolve_indentation(root)
    for dep_type in dependency_order:
        for dep_elem, dep_comments in dependencies_with_comments[dep_type]:
            for comment in dep_comments:
                root.remove(comment)
            root.remove(dep_elem)

    non_empty_order = [
        dep for dep in dependency_order if dependencies_with_comments[dep]
    ]
    insert_index = 0
    for index, dep_type in enumerate(non_empty_order):
        sorted_elems = sorted(
            dependencies_with_comments[dep_type], key=lambda x: x[0].text or ""
        )
        for i, elem_with_comment in enumerate(sorted_elems):
            is_last_in_group = i == len(sorted_elems) - 1
            next_group_starts_block = (
                index + 1 < len(non_empty_order)
                and non_empty_order[index + 1] in NEW_LINE_BEFORE
            )
            elem_with_comment[0].tail = (
                "\n\n" + indentation
                if is_last_in_group and next_group_starts_block
                else NEW_LINE + indentation
            )
            for comment in elem_with_comment[1]:
                root.insert(insert_index, comment)
                insert_index += 1
            root.insert(insert_index, elem_with_comment[0])
            insert_index += 1
    root[-1].tail = NEW_LINE


def check_for_duplicates(
    root: XmlElement,
    xml_file: str,
    check_only: bool,
    logger: logging.Logger,
) -> bool:
    """Detect (and optionally remove) duplicate elements with identical content."""
    seen: set[bytes] = set()
    duplicates: list[XmlElement] = []
    for elem in root:
        if elem.tag is ET.Comment:  # type: ignore[comparison-overlap]
            continue  # type: ignore[unreachable]
        combined = ET.tostring(elem, with_tail=False)
        if combined in seen:
            duplicates.append(elem)
        else:
            seen.add(combined)

    if duplicates:
        logger.info(
            f"Duplicate elements found in {xml_file}: {', '.join([elem.tag for elem in duplicates])}"
        )
        if check_only:
            return False

    if check_only:
        return True

    if not duplicates:
        return True

    for elem in duplicates:
        root.remove(elem)
    return False


def check_element_occurrences(
    root: XmlElement,
    xml_file: str,
    check_only: bool,
    logger: logging.Logger,
) -> bool:
    """Verify each element's occurrence count matches the schema bounds.

    Returns ``True`` when the tree is valid, ``False`` otherwise. Use
    :func:`check_element_occurrences_detailed` to distinguish ``FIXED``
    from ``UNFIXABLE`` failures.
    """
    return (
        check_element_occurrences_detailed(root, xml_file, check_only, logger)
        is CheckOutcome.VALID
    )


def check_element_occurrences_detailed(
    root: XmlElement,
    xml_file: str,
    check_only: bool,
    logger: logging.Logger,
) -> CheckOutcome:
    """Like :func:`check_element_occurrences` but distinguishes outcomes.

    Returns:
        :attr:`CheckOutcome.VALID` if the tree already conforms,
        :attr:`CheckOutcome.FIXED` if an issue was found and successfully
        corrected (only possible when ``check_only=False``),
        :attr:`CheckOutcome.UNFIXABLE` if an issue was found that the
        check cannot repair (e.g. a missing required element, or
        duplicates with non-identical content).
    """
    incorrect_occurrences = False
    has_unfixable = False
    for elem, min_occurrences, max_occurrences in ELEMENTS:
        count = len(root.findall(elem))
        if count < min_occurrences:
            logger.info(
                f"Error: Element <{elem}> in {xml_file} has fewer than {min_occurrences} occurrences."
            )
            incorrect_occurrences = True
            # A missing required element can never be auto-filled here
            # — we have no value to put in it.
            has_unfixable = True
        elif max_occurrences is not None and count > max_occurrences:
            logger.info(
                f"Error: Element <{elem}> in {xml_file} has more than {max_occurrences} occurrences."
            )
            incorrect_occurrences = True

    if not incorrect_occurrences:
        return CheckOutcome.VALID

    if check_only:
        # In check-only mode we can't repair anything; classify the same
        # finding the writeback path would have classified.
        if has_unfixable:
            return CheckOutcome.UNFIXABLE
        # Any over-occurrence found in check-only is presumed unfixable
        # without inspecting the bytes; surface it as a reportable failure.
        return CheckOutcome.UNFIXABLE

    # Try to correct the occurrences
    fixed_anything = False
    for elem, min_occurrence, max_occurrences in ELEMENTS:
        elements = root.findall(elem)
        count = len(elements)
        if max_occurrences is not None and count > max_occurrences:
            reference = (
                ET.tostring(elements[0], with_tail=False) if len(elements) > 0 else None
            )
            if reference and all(
                ET.tostring(elm, with_tail=False) == reference for elm in elements[1:]
            ):
                for extra in elements[max_occurrences:]:
                    root.remove(extra)
                logger.info(
                    f"Removed identical duplicate element <{elem}> from {xml_file}."
                )
                fixed_anything = True
            else:
                logger.warning(
                    f"Multiple <{elem}> entries differ. Please resolve manually. There are {count} occurrences in {xml_file} but there should be no more than {max_occurrences}."
                )
                has_unfixable = True
        if count < min_occurrence:
            logger.warning(f"Please add the missing element: <{elem}>")
            has_unfixable = True

    if has_unfixable:
        return CheckOutcome.UNFIXABLE
    return CheckOutcome.FIXED if fixed_anything else CheckOutcome.VALID


def check_element_order(
    root: XmlElement,
    xml_file: str,
    check_only: bool,
    logger: logging.Logger,
) -> bool:
    """Check (and optionally restore) the schema-defined element ordering."""
    element_order = [elem[0] for elem in ELEMENTS]

    current_order = [elem for elem in root if elem.tag in element_order]
    misplaced_elements = []
    for i, elem in enumerate(current_order):
        if i > 0 and element_order.index(elem.tag) < element_order.index(
            current_order[i - 1].tag
        ):
            misplaced_elements.append(elem)
    if misplaced_elements:
        logger.error(f"Element order in {xml_file} is incorrect.")
        logger.error(
            f"Misplaced elements: {', '.join([elem.tag for elem in misplaced_elements])}"
        )
        if check_only:
            return False

    if check_only:
        return True

    if not misplaced_elements:
        return True

    def _sort_key(elem: XmlElement) -> int:
        try:
            return element_order.index(elem.tag)
        except ValueError:
            # Place unexpected elements at the end
            return len(element_order)

    # Extract elements and their preceding comments
    elements_with_comments: list[tuple[XmlElement, list[XmlElement]]] = []
    current_comments: list[XmlElement] = []

    last_tail = ""
    indentation = resolve_indentation(root)
    for elem in root:
        if elem.tag is ET.Comment:  # type: ignore[comparison-overlap]
            logger.error(f"Found comment: {elem.text}")  # type: ignore[unreachable]
            if last_tail and last_tail[-1] == NEW_LINE:
                # inline comment -> append to previous element
                elements_with_comments[-1][0].tail = elem.tail
                elem.tail = NEW_LINE + indentation
                elements_with_comments[-1][1].append(deepcopy(elem))
            else:
                # Ensure only one NEW_LINE in elem.tail
                elem.tail = NEW_LINE + (
                    elem.tail.replace(NEW_LINE, "") if elem.tail else ""
                )
                current_comments.append(deepcopy(elem))
        else:
            elements_with_comments.append((deepcopy(elem), current_comments))
            current_comments = []
        last_tail = elem.tail if elem.tail else ""

    elements_with_comments.sort(key=lambda x: _sort_key(x[0]))

    # Clear the root and reinsert elements with their comments
    for elem in root:
        root.remove(elem)
    for elem, comments in elements_with_comments:
        for comment in comments:
            root.append(comment)
            logger.debug(f"Reinserted comment: {comment.text}")
        root.append(elem)
        logger.debug(f"Reinserted element: {elem.tag}")

    return False


def check_for_empty_lines(
    root: XmlElement,
    xml_file: str,
    check_only: bool,
    logger: logging.Logger,
) -> bool:
    """Ensure at most one blank line separates sibling elements."""
    found_empty_lines = False
    for elm in root:
        if elm.tail and elm.tail.count(NEW_LINE) > 2:
            logger.info(f"Error: More than one empty line found in {xml_file}.")
            found_empty_lines = True
            if check_only:
                return False
        if elm.tail is None or elm.tail.count(NEW_LINE) == 0:
            found_empty_lines = True
            logger.info(f"Error: Two elements are on the same line in {xml_file}.")
            if check_only:
                return False

    if check_only:
        return True
    if not found_empty_lines:
        return True
    indentation = resolve_indentation(root)
    for elm in root:
        if elm.tail and elm.tail.count(NEW_LINE) > 2:
            elm.tail = _remove_inner_newlines(elm.tail)
        elif elm.tail and elm.tail.count(NEW_LINE) == 0:
            elm.tail += NEW_LINE
        elif elm.tail is None:
            elm.tail = NEW_LINE + indentation
    return False


def check_indentation(
    root: XmlElement,
    check_only: bool,
    logger: logging.Logger,
    level: int = 1,
    indentation: str = "  ",
) -> bool:
    """Recursively validate indentation; correct it when ``check_only`` is False."""
    is_correct = True

    root.text, corrected = _check_and_correct(
        root.text, indentation * level, check_only
    )
    is_correct &= not corrected

    for index, elem in enumerate(root):
        is_last = index == len(root) - 1
        expected_indent = indentation * (level - 1) if is_last else indentation * level
        elem.tail, corrected = _check_and_correct(
            elem.tail, expected_indent, check_only
        )
        is_correct &= not corrected
        if len(elem) > 0:
            if not check_indentation(elem, check_only, logger, level + 1, indentation):
                is_correct = False
        else:
            if elem.text and NEW_LINE in elem.text:
                logger.error(
                    f"Element '{elem.tag}' has new lines in its text: '{elem.text}'"
                )
                is_correct = False
                if not check_only:
                    elem.text = elem.text.replace(NEW_LINE, " ").strip()
    if not is_correct and check_only:
        logger.error(
            "Incorrect indentation found in package.xml. Please fix the indentations."
        )
    elif not is_correct:
        logger.warning("Auto-corrected indentation in package.xml.")
    return is_correct


def check_for_non_existing_tags(
    root: XmlElement,
    xml_file: str,
    logger: logging.Logger,
) -> bool:
    """Reject tags that are not part of the package.xml schema."""
    non_existing_tags = []
    valid_tags = [e[0] for e in ELEMENTS]
    for elem in root:
        if isinstance(elem.tag, str) and elem.tag not in valid_tags:
            non_existing_tags.append(elem.tag)
    if non_existing_tags:
        logger.error(
            f"Unknown tags found in {xml_file}: {', '.join(non_existing_tags)}"
        )
        return False
    return True


# --- Module-private helpers ---


def _remove_inner_newlines(s: str) -> str:
    """Collapse multiple inner newlines to a single blank line."""
    first_newline_pos = s.find(NEW_LINE)
    last_newline_pos = s.rfind(NEW_LINE)

    if first_newline_pos == -1 or first_newline_pos == last_newline_pos:
        return s

    start = s[: first_newline_pos + 1]
    middle = s[first_newline_pos + 1 : last_newline_pos].replace(NEW_LINE, "")
    end = s[last_newline_pos:]
    return start + middle + end


def _check_indentation_string(string: str | None, expected_indent: str) -> bool:
    if not string or not isinstance(string, str):
        return False
    parsed_indentation = string.replace(NEW_LINE, "")
    return parsed_indentation == expected_indent and NEW_LINE in string


def _fix_indentation(string: str | None, expected_indent: str) -> str:
    indent = string.replace(" ", "") if string and NEW_LINE in string else NEW_LINE
    return indent + expected_indent


def _check_and_correct(
    string: str | None, expected_indent: str, check_only: bool
) -> tuple[str | None, bool]:
    if not _check_indentation_string(string, expected_indent):
        if not check_only:
            string = _fix_indentation(string, expected_indent)
        return string, True
    return string, False
