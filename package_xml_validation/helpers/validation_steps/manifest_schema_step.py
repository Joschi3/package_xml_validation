"""REP-149 manifest-invariant validation step."""

from __future__ import annotations

import os
import re
from typing import TYPE_CHECKING

from ._base import ValidationResult, ValidationStep

if TYPE_CHECKING:
    from ..package_types import XmlElement


_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")
_VERSION_RE = re.compile(r"^\d+\.\d+\.\d+$")
_SUPPORTED_FORMATS = frozenset({"1", "2", "3"})


class ManifestSchemaStep(ValidationStep):
    """Validate REP-149 structural invariants on the parsed tree.

    Rules:
      - ``<package format="3">`` is required. Missing or unsupported value
        is an error; ``"1"`` or ``"2"`` is a warning (older formats are
        still accepted by ROS but new packages should use 3).
      - ``<name>`` text matches ``^[a-z][a-z0-9_]*$``.
      - ``<version>`` text matches ``^\\d+\\.\\d+\\.\\d+$``.
      - Each ``<maintainer>`` has a non-empty ``email=`` attribute.

    Read-only: never mutates the tree. None of these can be safely auto-
    filled (we have no value to put in for a missing name/version, and
    fabricating a maintainer email would be misleading). Skipped when
    ``missing_deps_only`` or ``ignore_formatting_errors`` is set.
    """

    name = "Manifest schema"

    def perform_check(self, root: XmlElement, xml_file: str) -> ValidationResult:
        """Check REP-149 structural invariants."""
        result = ValidationResult(root=root)
        if self.config.missing_deps_only or self.config.ignore_formatting_errors:
            return result

        pkg_name = os.path.basename(os.path.dirname(xml_file))
        location = f"{pkg_name}/package.xml"

        self._check_format(root, location, result)
        self._check_name(root, location, result)
        self._check_version(root, location, result)
        self._check_maintainer_emails(root, location, result)

        if result.errors:
            result.valid = False
        return result

    @staticmethod
    def _check_format(
        root: XmlElement, location: str, result: ValidationResult
    ) -> None:
        fmt = root.get("format")
        if fmt is None:
            result.errors.append(
                f'Missing format="…" attribute on <package> in {location}. '
                'REP-149 requires format="3".'
            )
            return
        if fmt not in _SUPPORTED_FORMATS:
            result.errors.append(
                f'Unsupported format="{fmt}" on <package> in {location}. '
                'Expected "1", "2", or "3".'
            )
            return
        if fmt != "3":
            result.warnings.append(
                f'<package format="{fmt}"> in {location}: consider upgrading to '
                'format="3" (REP-149).'
            )

    @staticmethod
    def _check_name(root: XmlElement, location: str, result: ValidationResult) -> None:
        name_elem = root.find("name")
        if name_elem is None or not name_elem.text:
            # check_element_occurrences already reports missing <name>; don't
            # double-report here.
            return
        text = name_elem.text.strip()
        if not _NAME_RE.match(text):
            result.errors.append(
                f"Invalid <name>{text}</name> in {location}: must match "
                f"{_NAME_RE.pattern} (REP-149)."
            )

    @staticmethod
    def _check_version(
        root: XmlElement, location: str, result: ValidationResult
    ) -> None:
        version_elem = root.find("version")
        if version_elem is None or not version_elem.text:
            return
        text = version_elem.text.strip()
        if not _VERSION_RE.match(text):
            result.errors.append(
                f"Invalid <version>{text}</version> in {location}: must match "
                f"MAJOR.MINOR.PATCH with numeric parts only (REP-149)."
            )

    @staticmethod
    def _check_maintainer_emails(
        root: XmlElement, location: str, result: ValidationResult
    ) -> None:
        for maintainer in root.findall("maintainer"):
            email = maintainer.get("email")
            if not email or not email.strip():
                who = (maintainer.text or "").strip() or "<unnamed>"
                result.errors.append(
                    f"<maintainer>{who}</maintainer> in {location} is missing "
                    'a non-empty email="…" attribute (REP-149).'
                )
