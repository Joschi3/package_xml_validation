#!/usr/bin/env python3
"""
finds all packages in a ROS2 workspace given a path to a package, a file in a pkg or the workspace root.

Usage
-----
$ python3 find_ros_ws_pkgs.py /abs/path/to/some_pkg    # prints package names
$ python3 find_ros_ws_pkgs.py --full-paths .           # prints names + paths

"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import os


def find_package_dir(path: Path) -> Path:
    """Return a directory that contains *package.xml* according to the order:

    1. the path itself (or its parent if it is a file)
    2. the first package found **within** the path
    3. the first ancestor that is a package

    Raises ``ValueError`` if no package can be located.

    Args:
        path: File or directory to search from.

    Returns:
        Path to a directory containing package.xml.

    """
    p = path.resolve()

    # Normalise: if the user passed a file, start with its directory
    if p.is_file():
        p = p.parent

    # 1. path itself
    if (p / "package.xml").is_file():
        return p

    # 2. search downward
    pkg_xml_paths = list(p.rglob("package.xml"))
    # make sure no 'build' or 'install' folders are included
    pkg_xml_paths = [
        xml
        for xml in pkg_xml_paths
        if "build" not in xml.parts and "install" not in xml.parts
    ]
    for xml in pkg_xml_paths:
        return xml.parent  # first match wins

    # 3. search upward
    for ancestor in p.parents:
        if (ancestor / "package.xml").is_file():
            return ancestor

    raise ValueError(
        f"No *package.xml* found relative to '{path}'. Is it really part of a "
        "ROS 2 workspace?"
    )


def looks_like_ws_root(path: Path, original_pkg: Path) -> bool:
    """Check whether a path looks like a ROS workspace root.

    Args:
        path: Candidate workspace root directory.
        original_pkg: Original package path to verify is under <ws>/src.

    Returns:
        True if the path looks like a workspace root, otherwise False.

    """
    src_dir = path / "src"
    return src_dir.is_dir() and original_pkg.is_relative_to(src_dir)


def find_workspace_root(start: Path) -> Path:
    """Walk upwards until a workspace root is found.

    Args:
        start: Starting path (file or directory).

    Returns:
        Path to the workspace root directory.

    """
    here = start.resolve()
    here = here.parent if here.is_file() else here  # deal with file paths

    while True:
        if looks_like_ws_root(here, start):
            return here
        if here == here.parent:  # reached filesystem root
            break
        here = here.parent

    raise ValueError(
        f"Could not locate a ROS 2 workspace root above {start}. "
        "Does the path actually reside in a <ws>/src/<pkg> hierarchy?"
    )


def parse_pkg_name(package_xml: Path) -> str:
    """Return the <name> tag from package.xml, or fall back to folder name.

    Args:
        package_xml: Path to a package.xml file.

    Returns:
        Package name as a string.

    """
    try:
        tag = ET.parse(package_xml).find("name")
        if tag is not None and tag.text:
            return tag.text.strip()
    except ET.ParseError:
        pass
    return package_xml.parent.name  # fallback


def pkg_iterator(src_dir: Path) -> dict[str, Path]:
    """Collect package names and paths under a src directory.

    Args:
        src_dir: Path to the workspace src directory.

    Returns:
        Mapping of package name to package directory path.

    """
    pkgs: dict[str, Path] = {}
    for xml in src_dir.rglob("package.xml"):
        # Respect COLCON_IGNORE: ignore a path if any ancestor contains the file
        if any(
            (parent / "COLCON_IGNORE").exists() or (parent / "colcon_ignore").exists()
            for parent in xml.parents
        ):
            continue
        pkgs[parse_pkg_name(xml)] = xml.parent
    return pkgs


def get_pkgs_in_wrs(path: Path) -> list[str]:
    """Return all package names in the workspace that contains *path*.

    Args:
        path: Path within a workspace (file or directory).

    Returns:
        Sorted list of package names.

    """
    if isinstance(path, str):
        path = Path(path).resolve(strict=True)
    if not path.exists():
        raise ValueError(f"Path does not exist: {path}")
    pkg_dir = None
    try:
        pkg_dir = find_package_dir(path)
        ws_root = find_workspace_root(pkg_dir)
        src_dir = ws_root / "src"
    except Exception as e:
        print(f"Exception extracting local pkgs: {e}")
        if pkg_dir and os.path.exists(pkg_dir.absolute()):
            print(f"Attempting to extract local pkgs from {pkg_dir}")
            src_dir = pkg_dir.parent
        else:
            print("Unable to extract local pkgs")
            return []
    pkgs = pkg_iterator(src_dir)
    return sorted(pkgs)


def _is_ignored_dir(path: Path) -> bool:
    """
    Return True if `path` or any ancestor contains an ignore marker:
    - 'coclon_ignore' (as requested)
    - 'COLCON_IGNORE' (colcon's standard)

    Args:
        path: Directory path to inspect.

    Returns:
        True if any ancestor contains an ignore marker, otherwise False.

    """
    path = path.resolve()
    for parent in (path, *path.parents):
        if (parent / "coclon_ignore").exists() or (parent / "COLCON_IGNORE").exists():
            return True
    return False


def find_package_xml_files(paths) -> list[str]:
    """Find all package.xml files under the provided paths.

    Args:
        paths: Iterable of file or directory paths to search.

    Returns:
        Sorted list of package.xml paths as strings.

    """
    files: set[Path] = set()

    for p in paths:
        p = Path(p).resolve()
        if not p.exists():
            continue

        if p.is_file():
            if p.name == "package.xml":
                if not _is_ignored_dir(p.parent):
                    files.add(p)
            elif p.name == "CMakeLists.txt":
                xml = (p.parent / "package.xml").resolve()
                if xml.exists() and not _is_ignored_dir(xml.parent):
                    files.add(xml)

        elif p.is_dir():
            # Collect all package.xml files under p, but drop those under ignored trees
            for xml in p.rglob("package.xml"):
                if "build" in xml.parts or "install" in xml.parts:
                    continue
                if not _is_ignored_dir(xml.parent):
                    files.add(xml.resolve())

    return sorted(str(x) for x in files)


def main() -> None:
    """CLI entrypoint for listing packages in a workspace.

    Args:
        None.

    Returns:
        None.

    """
    ap = argparse.ArgumentParser(
        description=(
            "Locate the ROS2 workspace for *path* and list every package in it. "
            "The path may be a package, the src folder, or the workspace root."
        )
    )
    ap.add_argument("path", help="File or directory anywhere in the workspace")
    ap.add_argument(
        "-f",
        "--full-paths",
        action="store_true",
        help="Print each package's path in addition to its name",
    )
    args = ap.parse_args()

    try:
        start_path = Path(args.path).resolve(strict=True)
    except FileNotFoundError:
        sys.exit(f"ERROR: {args.path} does not exist")

    try:
        pkg_dir = find_package_dir(start_path)  # << NEW
        ws_root = find_workspace_root(pkg_dir)
    except ValueError as e:
        sys.exit(f"ERROR: {e}")

    pkgs = pkg_iterator(ws_root / "src")
    if not pkgs:
        sys.exit("WARNING: no packages found under <ws>/src")

    print(f"Workspace: {ws_root}")
    for name, path in sorted(pkgs.items()):
        print(f"{name} {path}" if args.full_paths else name)


if __name__ == "__main__":
    main()
