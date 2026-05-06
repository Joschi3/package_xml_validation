#!/usr/bin/env python3
"""
find_launch_dependencies.py

Recursively search a ROS 2 package's launch/ folder and extract
all referenced ROS 2 package names via a small set of regexes.

Now ignores matches that occur inside comments:
- Python: # line comments, and triple-quoted blocks
- XML/.launch: <!-- ... --> comments
- YAML: # line comments
"""

import logging
import os
import re

logger = logging.getLogger(__name__)


REGEX_EXPR = [
    # YAML-style:  pkg: <pkg_name>
    r"pkg\s*:\s*['\"]?([A-Za-z0-9_]+)['\"]?",
    # Hector launch component:  package: <pkg_name>
    r"package\s*:\s*['\"]?([A-Za-z0-9_]+)['\"]?",
    # Python Node-family constructors: Node / LifecycleNode / ComposableNode /
    # ComposableLifecycleNode (..., package='<pkg_name>', ...)
    r"(?:^|[^\w])(?:Node|LifecycleNode|ComposableNode|ComposableLifecycleNode)\s*\(\s*[^)]*?\bpackage\s*=\s*['\"]([A-Za-z0-9_]+)['\"]",
    # XML node tag: <node pkg="foo" ...>
    r"<node[^>]*?\bpkg\s*=\s*['\"]?([A-Za-z0-9_]+)['\"]?",
    # XML composable node: <composable_node pkg="foo" ...>
    r"<composable_node[^>]*?\bpkg\s*=\s*['\"]?([A-Za-z0-9_]+)['\"]?",
    # XML node container: <node_container pkg="foo" ...>
    r"<node_container[^>]*?\bpkg\s*=\s*['\"]?([A-Za-z0-9_]+)['\"]?",
    # get_package_share_directory('foo')
    r"get_package_share_directory\(\s*['\"]([A-Za-z0-9_]+)['\"]\s*\)",
    # get_package_share_path('foo')
    r"get_package_share_path\(\s*['\"]([A-Za-z0-9_]+)['\"]\s*\)",
    # get_package_prefix('foo')
    r"get_package_prefix\(\s*['\"]([A-Za-z0-9_]+)['\"]\s*\)",
    # FindPackageShare('foo')
    r"FindPackageShare\(\s*['\"]([A-Za-z0-9_]+)['\"]\s*\)",
    # FindPackageShare(package='foo')
    r"FindPackageShare\(\s*package\s*=\s*['\"]([A-Za-z0-9_]+)['\"]\s*\)",
    # FindPackagePrefix('foo') / FindPackagePrefix(package='foo')
    r"FindPackagePrefix\(\s*(?:package\s*=\s*)?['\"]([A-Za-z0-9_]+)['\"]\s*\)",
    # $(find-pkg-share foo)
    r"\$\(\s*find-pkg-share\s+([A-Za-z0-9_]+)\s*\)",
    # $(find-pkg-prefix foo)
    r"\$\(\s*find-pkg-prefix\s+([A-Za-z0-9_]+)\s*\)",
    # better_launch Python:  bl.node("pkg", ...)  /  bl.include("pkg", ...)
    r"\bbl\.node\(\s*['\"]([A-Za-z0-9_]+)['\"]",
    r"\bbl\.include\(\s*['\"]([A-Za-z0-9_]+)['\"]",
]

COMPILED = [re.compile(rx) for rx in REGEX_EXPR]

# TOML patterns are kept separate and only applied to .toml files under a
# launch/ directory, to avoid false positives from arbitrary TOML configs.
TOML_REGEX_EXPR = [
    # better_launch TOML:  package = "pkg"
    r"^\s*package\s*=\s*['\"]([A-Za-z0-9_]+)['\"]",
]

TOML_COMPILED = [re.compile(rx, re.MULTILINE) for rx in TOML_REGEX_EXPR]

_TRIPLE_QUOTE_BLOCK = re.compile(r"(?s)(['\"]{3})(?:.*?)(\1)")
_XML_COMMENT_BLOCK = re.compile(r"(?s)<!--.*?-->")


def _strip_hash_line_comments_outside_strings(text: str) -> str:
    """
    Remove '#' to end-of-line comments that occur OUTSIDE of single/double quoted strings.
    Preserves newlines.
    Suitable for Python and YAML after any triple-quoted removal (for Python).

    Args:
        text: Input text to process.

    Returns:
        Text with hash comments removed outside quoted strings.

    """
    out = []
    in_single = False
    in_double = False
    i = 0
    n = len(text)

    while i < n:
        ch = text[i]

        # Handle escapes inside strings
        if ch == "\\" and (in_single or in_double) and i + 1 < n:
            out.append(ch)
            out.append(text[i + 1])
            i += 2
            continue

        if not in_single and not in_double:
            if ch == "#":
                # skip until end of line (keep the newline itself)
                while i < n and text[i] not in ("\n", "\r"):
                    i += 1
                # fall through to append the newline (if any)
                continue
            elif ch == "'":
                in_single = True
                out.append(ch)
                i += 1
                continue
            elif ch == '"':
                in_double = True
                out.append(ch)
                i += 1
                continue
            else:
                out.append(ch)
                i += 1
                continue
        else:
            # inside quotes
            if in_single and ch == "'":
                in_single = False
            elif in_double and ch == '"':
                in_double = False
            out.append(ch)
            i += 1

    return "".join(out)


def _decomment_python(text: str) -> str:
    """Remove Python comments and triple-quoted blocks.

    Args:
        text: Python source text.

    Returns:
        Text with comments removed.

    """
    # 1) drop triple-quoted blocks entirely
    text = _TRIPLE_QUOTE_BLOCK.sub("", text)
    # 2) drop '#' comments outside of quoted strings
    text = _strip_hash_line_comments_outside_strings(text)
    return text


def _decomment_xml(text: str) -> str:
    """Remove XML comments.

    Args:
        text: XML source text.

    Returns:
        Text with XML comments removed.

    """
    return _XML_COMMENT_BLOCK.sub("", text)


def _decomment_yaml(text: str) -> str:
    """Remove YAML hash comments outside strings.

    Args:
        text: YAML source text.

    Returns:
        Text with comments removed.

    """
    return _strip_hash_line_comments_outside_strings(text)


def _decomment_toml(text: str) -> str:
    """Remove TOML hash comments outside strings.

    Args:
        text: TOML source text.

    Returns:
        Text with comments removed.

    """
    return _strip_hash_line_comments_outside_strings(text)


def _decomment_for_suffix(suffix: str, text: str) -> str:
    """Strip comments based on file suffix.

    Args:
        suffix: Filename or suffix to determine comment style.
        text: File contents.

    Returns:
        Text with comments removed where applicable.

    """
    s = suffix.lower()
    if s.endswith(".py"):
        return _decomment_python(text)
    if s.endswith(".xml") or s.endswith(".launch"):
        return _decomment_xml(text)
    if s.endswith(".yaml") or s.endswith(".yml"):
        return _decomment_yaml(text)
    if s.endswith(".toml"):
        return _decomment_toml(text)
    # default: no decommenting
    return text


def _is_under_launch_dir(path: str) -> bool:
    """Return True if any path component (excluding the basename) is named 'launch'.

    Args:
        path: File path to inspect.

    Returns:
        Whether the file lives under a 'launch' directory at any depth.

    """
    parts = os.path.normpath(path).split(os.sep)
    return "launch" in parts[:-1]


def scan_file(path: str, found: set[str], verbose: bool = False) -> None:
    """Scan a single launch file for package references.

    Args:
        path: File path to scan.
        found: Set to add discovered package names to.
        verbose: Whether to print verbose match details.

    Returns:
        None.

    """
    with open(path, encoding="utf-8") as f:
        text = f.read()

    text = _decomment_for_suffix(path, text)

    is_toml = path.lower().endswith(".toml")
    if is_toml:
        # Only honour TOML matches if the file is scoped under a launch/ dir,
        # to avoid false positives from arbitrary project TOML configs.
        if not _is_under_launch_dir(path):
            return
        patterns = TOML_COMPILED
        sources = TOML_REGEX_EXPR
    else:
        patterns = COMPILED
        sources = REGEX_EXPR

    for i, rx in enumerate(patterns):
        for m in rx.finditer(text):
            pkg = m.group(1)
            found.add(pkg)
            if verbose:
                logger.debug(
                    "Found package '%s' in %s with regex %s",
                    pkg,
                    os.path.basename(path),
                    sources[i],
                )


def scan_files(launch_dir: str, verbose: bool = False) -> list[str]:
    """
    Extracts launch dependencies from the specified directory.
    Launch dependencies are listed packages names in the launch files.
    It uses regex to extract package names from common launch patterns.
    Comments are stripped (type-specific) before matching.

    Args:
        launch_dir: Directory to scan recursively.
        verbose: Whether to print verbose match details.

    Returns:
        Sorted list of discovered package names.

    """
    if not os.path.isdir(launch_dir):
        logger.error("'%s' is not a directory.", launch_dir)
        return []

    pkgs: set[str] = set()

    for root, _, files in os.walk(launch_dir):
        for fn in files:
            full = os.path.join(root, fn)
            if fn.endswith((".py", ".xml", ".yaml", ".launch", ".yml")):
                scan_file(full, pkgs, verbose)
            elif fn.endswith(".toml") and _is_under_launch_dir(full):
                scan_file(full, pkgs, verbose)
    return sorted(pkgs)
