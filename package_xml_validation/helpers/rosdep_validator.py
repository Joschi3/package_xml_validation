import re
import yaml
import difflib
from importlib import resources

# ROS imports
import rosdep2
from rosdep2.sources_list import (
    SourcesListLoader,
    CachedDataSource,
    get_sources_cache_dir,
)

# Try importing regex for fuzzy matching
try:
    import regex

    HAS_REGEX_MODULE = True
except ImportError:
    HAS_REGEX_MODULE = False

try:
    from .workspace import get_pkgs_in_wrs
except ImportError:

    def get_pkgs_in_wrs(path):
        return []


class RosdepValidator:
    """
    Class to validate ROS dependencies using rosdep.
    """

    def __init__(self, pkg_path=None):
        self.rosdep_installer_context = rosdep2.create_default_installer_context()
        self.rosdep = rosdep2.RosdepLookup.create_from_rospkg()

        (
            self.rosdep_os_name,
            self.rosdep_os_version,
        ) = self.rosdep_installer_context.get_os_name_and_version()

        self.rosdep_view = self.rosdep.get_rosdep_view(
            rosdep2.rospkg_loader.DEFAULT_VIEW_KEY
        )

        self.sources_loader = SourcesListLoader.create_default(
            sources_cache_dir=get_sources_cache_dir(), verbose=False
        )

        self.local_pkgs = []
        if pkg_path:
            self.local_pkgs = get_pkgs_in_wrs(pkg_path)

        self.cmake_rosdep_map = self._load_cmake_rosdep_map()

    def _load_cmake_rosdep_map(self) -> dict[str, str]:
        try:
            map_file = resources.files("package_xml_validation").joinpath(
                "data/cmake_rosdep_map.yaml"
            )
            with map_file.open("r", encoding="utf-8") as handle:
                mapping = yaml.safe_load(handle) or {}
        except Exception:
            return {}

        if not isinstance(mapping, dict):
            return {}
        return {str(k).lower(): str(v) for k, v in mapping.items()}

    def can_resolve(self, dependency: str) -> bool:
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
        except (rosdep2.ResolutionError, KeyError):
            return False
        except Exception:
            return False

    def resolve_cmake_dependency(self, dependency: str) -> str | None:
        if dependency in self.local_pkgs:
            return dependency
        if self.can_resolve(dependency):
            return dependency
        mapped = self.cmake_rosdep_map.get(dependency.lower())
        if mapped:
            return mapped
        return None

    def search_rosdep_candidates(self, dependency: str) -> list[str]:
        """
        Searches the local rosdep cache for keys that loosely match the dependency.
        Returns sorted results: Exact -> Contains(Full) -> Contains(Stem) -> Fuzzy.
        """
        regexes = []
        if HAS_REGEX_MODULE:
            # Fuzzy matching: allow 2 errors for long strings, 1 for short
            error_tolerance = 2 if len(dependency) >= 7 else 1
            pattern = rf"(?:{re.escape(dependency)}){{e<={error_tolerance}}}"
            regexes.append(regex.compile(pattern, regex.BESTMATCH | regex.IGNORECASE))
        else:
            # Fallback: standard regex
            regexes.append(re.compile(re.escape(dependency), re.IGNORECASE))

        found_keys = []

        # Iterate over cached views
        for view_name in self.sources_loader.get_loadable_views():
            try:
                view = self.sources_loader.get_source(view_name=view_name)
            except Exception:
                continue

            # Skip remote sources
            if not isinstance(view, CachedDataSource) or not hasattr(
                view, "rosdep_data"
            ):
                continue

            matches = self._search_view_data(view, regexes)
            found_keys.extend(matches)

        unique_keys = list(set(found_keys))

        # --- IMPROVED SORTING LOGIC ---
        def sort_key(candidate):
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

    def _search_view_data(self, view, compiled_regexes) -> list[str]:
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

    def _check_payload_match(self, os_payload, compiled_regexes) -> bool:
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

    def check_rosdeps(self, dependencies) -> list[str]:
        unresolvable = []
        for dep in dependencies:
            if not self.can_resolve(dep):
                unresolvable.append(dep)
        return unresolvable

    def check_rosdeps_and_local_pkgs(self, dependencies) -> list[str]:
        unresolvable_rosdeps = self.check_rosdeps(dependencies)
        final_missing = [
            dep for dep in unresolvable_rosdeps if dep not in self.local_pkgs
        ]
        return final_missing
