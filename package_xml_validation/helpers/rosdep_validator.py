from __future__ import annotations

import re
import yaml
import difflib
from importlib import resources
from typing import Any
from collections.abc import Iterable

from . import rosdep_wrapper

# Try importing regex for fuzzy matching
try:
    import regex

    HAS_REGEX_MODULE = True
except ImportError:
    HAS_REGEX_MODULE = False

from .workspace import get_pkgs_in_wrs


class RosdepValidator:
    """
    Class to validate ROS dependencies using rosdep.
    """

    def __init__(self, pkg_path: str | None = None) -> None:
        """Initialize rosdep lookup and optional workspace package list.

        Args:
            pkg_path: Optional path used to discover local workspace packages.

        Returns:
            None.

        """
        self.rosdep_installer_context = rosdep_wrapper.create_installer_context()
        self.rosdep = rosdep_wrapper.create_lookup_from_rospkg()

        (
            self.rosdep_os_name,
            self.rosdep_os_version,
        ) = self.rosdep_installer_context.get_os_name_and_version()

        self.rosdep_view = self.rosdep.get_rosdep_view(
            rosdep_wrapper.get_default_view_key()
        )

        self.sources_loader = rosdep_wrapper.create_default_sources_loader(
            verbose=False
        )

        self._cached_data_source_cls = rosdep_wrapper.get_cached_data_source_cls()

        self.local_pkgs: list[str] = []
        if pkg_path:
            self.local_pkgs = get_pkgs_in_wrs(pkg_path)

        self.cmake_rosdep_map = self._load_cmake_rosdep_map()

    def _load_cmake_rosdep_map(self) -> dict[str, str]:
        """Load the CMake-to-rosdep mapping file from package data.

        Args:
            None.

        Returns:
            A dictionary mapping CMake dependency names to rosdep keys.

        """
        try:
            map_file = resources.files("package_xml_validation").joinpath(
                "data/cmake_rosdep_map.yaml"
            )
            with map_file.open("r", encoding="utf-8") as handle:
                mapping = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError):
            return {}

        if not isinstance(mapping, dict):
            return {}
        return {str(k).lower(): str(v) for k, v in mapping.items()}

    def can_resolve(self, dependency: str) -> bool:
        """Check whether a rosdep key resolves for the current OS.

        Args:
            dependency: Rosdep key to resolve.

        Returns:
            True if resolvable for the current OS, otherwise False.

        """
        try:
            dep = self.rosdep_view.lookup(dependency)
            installer, _ = dep.get_rule_for_platform(
                self.rosdep_os_name,
                self.rosdep_os_version,
                self.rosdep_installer_context.get_os_installer_keys(
                    self.rosdep_os_name
                ),
                self.rosdep_installer_context.get_default_os_installer_key(
                    self.rosdep_os_name
                ),
            )
            return installer is not None
        except (rosdep_wrapper.ResolutionError, KeyError):
            return False

    def resolve_cmake_dependency(self, dependency: str) -> str | None:
        """Resolve a CMake dependency to a rosdep key or local package name.

        Args:
            dependency: CMake dependency name.

        Returns:
            A resolvable rosdep key or local package name, or None if unresolved.

        """
        if dependency in self.local_pkgs:
            return dependency
        if self.can_resolve(dependency):
            return dependency
        mapped = self.cmake_rosdep_map.get(dependency.lower())
        if mapped:
            return mapped
        return None

    def search_rosdep_candidates(self, dependency: str) -> list[str]:
        """Search rosdep cache for candidate keys related to a dependency name.

        Args:
            dependency: Dependency name to match against rosdep keys.

        Returns:
            A list of up to five candidate rosdep keys, sorted by relevance.

        """
        # ``regex`` and ``re`` produce distinct (but duck-compatible) Pattern
        # types. Annotated as Any to let either populate the list.
        regexes: list[Any] = []
        if HAS_REGEX_MODULE:
            # Fuzzy matching: allow 2 errors for long strings, 1 for short
            error_tolerance = 2 if len(dependency) >= 7 else 1
            pattern = rf"(?:{re.escape(dependency)}){{e<={error_tolerance}}}"
            regexes.append(regex.compile(pattern, regex.BESTMATCH | regex.IGNORECASE))
        else:
            # Fallback: standard regex
            regexes.append(re.compile(re.escape(dependency), re.IGNORECASE))

        found_keys: list[str] = []

        # Iterate over cached views
        for view_name in self.sources_loader.get_loadable_views():
            try:
                view = self.sources_loader.get_source(view_name=view_name)
            except rosdep_wrapper.ResourceNotFound:
                continue

            # Skip remote sources
            if not isinstance(view, self._cached_data_source_cls) or not hasattr(
                view, "rosdep_data"
            ):
                continue

            matches = self._search_view_data(view, regexes)
            found_keys.extend(matches)

        unique_keys = list(set(found_keys))

        # --- IMPROVED SORTING LOGIC ---
        def sort_key(candidate: str) -> tuple[int, float]:
            cand_lower = candidate.lower()
            dep_lower = dependency.lower()

            # Tier 0: Exact Match
            if cand_lower == dep_lower:
                return (0, 0)

            # Tier 1: Contains the FULL dependency
            # e.g. 'ecl_threads' contains 'threads'
            if dep_lower in cand_lower:
                # Sort by length (shorter containing string is likely more relevant)
                return (1, len(candidate))

            # Tier 2: Contains the SINGULAR dependency (Stemming)
            # e.g. 'libboost-thread' contains 'thread' (from search 'Threads')
            if dep_lower.endswith("s") and dep_lower[:-1] in cand_lower:
                return (2, len(candidate))

            # Tier 3: Fuzzy Similarity (difflib)
            # Use negative ratio to sort descending (highest similarity first)
            similarity = difflib.SequenceMatcher(None, dep_lower, cand_lower).ratio()
            return (3, -similarity)

        return sorted(unique_keys, key=sort_key)[:5]

    def _search_view_data(self, view: Any, compiled_regexes: list[Any]) -> list[str]:
        """Search a rosdep view for matching keys or payload values.

        Args:
            view: Rosdep view/source to search.
            compiled_regexes: List of compiled regexes to match.

        Returns:
            Matching rosdep keys found in the view.

        """
        matches = []
        for key, item in view.rosdep_data.items():
            if any(r.search(key) for r in compiled_regexes):
                matches.append(key)
                continue
            if self.rosdep_os_name in item:
                os_entry = item[self.rosdep_os_name]
                if self._check_payload_match(os_entry, compiled_regexes):
                    matches.append(key)
        return matches

    def _check_payload_match(
        self, os_payload: Any, compiled_regexes: list[Any]
    ) -> bool:
        """Check whether a payload contains any regex matches.

        Args:
            os_payload: Rosdep payload entry (str/list/dict).
            compiled_regexes: List of compiled regexes to match.

        Returns:
            True if any payload field matches, otherwise False.

        """
        if isinstance(os_payload, str):
            return any(r.search(os_payload) for r in compiled_regexes)
        elif isinstance(os_payload, list):
            for pkg in os_payload:
                if isinstance(pkg, str) and any(
                    r.search(pkg) for r in compiled_regexes
                ):
                    return True
        elif isinstance(os_payload, dict):
            if "packages" in os_payload:
                return self._check_payload_match(
                    os_payload["packages"], compiled_regexes
                )
            for sub_val in os_payload.values():
                if self._check_payload_match(sub_val, compiled_regexes):
                    return True
        return False

    def check_rosdeps(self, dependencies: Iterable[str]) -> list[str]:
        """Return a list of rosdep keys that cannot be resolved.

        Args:
            dependencies: Iterable of rosdep keys to validate.

        Returns:
            List of unresolvable rosdep keys.

        """
        unresolvable = []
        for dep in dependencies:
            if not self.can_resolve(dep):
                unresolvable.append(dep)
        return unresolvable

    def check_rosdeps_and_local_pkgs(self, dependencies: Iterable[str]) -> list[str]:
        """Return rosdep keys unresolved and not satisfied by local packages.

        Args:
            dependencies: Iterable of rosdep keys to validate.

        Returns:
            List of unresolved keys excluding known local packages.

        """
        unresolvable_rosdeps = self.check_rosdeps(dependencies)
        final_missing = [
            dep for dep in unresolvable_rosdeps if dep not in self.local_pkgs
        ]
        return final_missing
