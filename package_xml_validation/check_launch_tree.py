#!/usr/bin/env python3
"""Does everything this launch tree reaches actually exist here?

Follows every include from a package's launch files, across package boundaries, and reports
two kinds of finding:

* **a package that is not installed** - the launch file names it, nothing here provides it;
* **an include that leads nowhere** - the package is there, the launch file it names is not.

Neither is fixable by editing a manifest, so this is a separate entry point from
`package_xml_validator` rather than a flag on it: it only reports, and it resolves through
`AMENT_PREFIX_PATH`, so it needs a built and sourced workspace. Without one it says so and
skips, rather than reporting every package as missing.
"""

from typing import Optional
import argparse
import os
import sys

from .helpers.find_launch_dependencies import (
    Walk,
    default_share_lookup,
    scan_directory,
)
from .helpers.workspace import find_package_xml_files

__all__ = ["check", "main", "package_dirs_under"]


#: Directories worth seeding from: `launch`, plus the two that hold launch-manager component
#: definitions naming the package and launch file they start.
DEFAULT_SEED_DIRS = ("launch", "launch_manager_components", "launch_manager_configs")


def workspace_is_available() -> bool:
    """Whether there is anything to resolve packages against."""
    return any(
        part for part in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    )


def _installed(package: str) -> bool:
    return default_share_lookup(package) is not None


def package_dirs_under(paths: "list[str]") -> "list[str]":
    """The directories holding a `package.xml`, at any depth below `paths`.

    `check` works on one package at a time, but the hook is given a repository root. Reusing
    the validator's discovery means both entry points agree on what a package is, and on which
    directories (`COLCON_IGNORE` and friends) are skipped.
    """
    return sorted({os.path.dirname(one) for one in find_package_xml_files(paths)})


def check(package_dir: str, args: Optional[dict] = None) -> "tuple[Walk, list, list]":
    """Follow one package's launch tree.

    `package_dir` is a single package directory, not a repository root - see
    :py:func:`package_dirs_under`. Returns `(walk, missing packages, dead includes)`.
    """
    seeds = [
        os.path.join(package_dir, name)
        for name in DEFAULT_SEED_DIRS
        if os.path.isdir(os.path.join(package_dir, name))
    ]

    references: list = []
    edges: list = []
    visited: list = []
    seen: set = set()

    for seed in seeds:
        walk = scan_directory(seed, args=args)
        references.extend(walk.references)
        edges.extend(walk.edges)
        for one in walk.visited:
            if one not in seen:
                seen.add(one)
                visited.append(one)

    walk = Walk(references=references, edges=edges, visited=visited)

    missing = sorted(
        {one.package for one in walk.references if not _installed(one.package)}
    )

    # An include unfollowable *because its package is absent* is already reported as missing.
    dead = [
        one for one in walk.unresolved if one.package and one.package not in missing
    ]

    return walk, missing, dead


def _where(path: str, package_dir: str) -> str:
    """A path a reader can act on.

    Relative while the file belongs to this package, absolute once it does not - a chain of
    `../..` out of the workspace and into `/opt` tells nobody anything.
    """
    relative = os.path.relpath(path, package_dir)
    return os.path.abspath(path) if relative.startswith("..") else relative


def _report(package_dir: str, walk: Walk, missing: list, dead: list) -> None:
    name = os.path.basename(os.path.abspath(package_dir))
    print(
        f"{name}: read {len(walk.visited)} launch file(s), {len(walk.packages)} package(s)"
    )

    where: dict = {}
    for one in walk.references:
        where.setdefault(one.package, one)

    for package in missing:
        cited = where.get(package)
        print(f"  {package} is not installed in this workspace")
        if cited:
            print(f"      referenced by {_where(cited.file, package_dir)}:{cited.line}")

    for edge in dead:
        print(f"  {edge.package} does not ship {edge.launch_file}")
        print(f"      referenced by {_where(edge.file, package_dir)}:{edge.line}")

    # Edges the two sections above have not already accounted for.
    explained = set(missing)
    unread = [
        one
        for one in walk.unresolved
        if one not in dead and one.package not in explained
    ]
    if unread:
        # Printed even when nothing is wrong, so a clean result that skipped half the tree
        # does not read like one that did not.
        print(f"  {len(unread)} include(s) could not be followed and were not checked:")
        for edge in unread[:5]:
            target = f"{edge.package or '(computed)'}/{edge.launch_file}"
            print(f"      {_where(edge.file, package_dir)}:{edge.line}  {target}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Check that everything a package's launch tree references exists here."
    )
    parser.add_argument(
        "src",
        nargs="*",
        default=["."],
        help="Files or directories to search for packages. Defaults to the current directory.",
    )
    parser.add_argument(
        "--error",
        action="store_true",
        help="Exit non-zero on findings. Warns only by default, so it can be introduced to a "
        "repository that already has some.",
    )
    parser.add_argument(
        "--arg",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="A value for $(var NAME), overriding the default a launch file declares. Repeat "
        "for several. Without these the tree is followed as its defaults describe it.",
    )
    parsed = parser.parse_args()

    if not workspace_is_available():
        # Not a failure: with nothing to resolve against, every package would read as missing.
        print(
            "check-launch-tree: AMENT_PREFIX_PATH is empty, so packages cannot be resolved - "
            "the launch tree was not checked. Source the workspace to enable this check."
        )
        return

    args = {}
    for item in parsed.arg:
        name, _, value = item.partition("=")
        if name:
            args[name] = value

    package_dirs = package_dirs_under(parsed.src)
    if not package_dirs:
        print("check-launch-tree: no packages found. Nothing to check.")
        return

    findings = 0
    for package_dir in package_dirs:
        walk, missing, dead = check(package_dir, args or None)
        _report(package_dir, walk, missing, dead)
        findings += len(missing) + len(dead)

    if findings and parsed.error:
        sys.exit(1)


if __name__ == "__main__":
    main()
