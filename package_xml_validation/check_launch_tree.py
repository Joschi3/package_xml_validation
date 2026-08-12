#!/usr/bin/env python3
"""Does everything this launch tree reaches exist, and does every manifest admit to it?

Follows every include out of a package's launch files, across package boundaries, and grades
what it finds by whether anyone can act on it:

* **a reference no manifest declares** - a launch file names a package that its own
  `package.xml` does not depend on. A defect in the checkout, visible without a workspace,
  so it fails the run.
* **an include that leads nowhere** - the package is installed, the launch file it names is
  not. Also a defect, and also fatal.
* **a package that is not installed here** - a warning. A CI runner may simply not have built
  it, and nothing in the checkout is wrong.
* **an include nobody could follow** - a warning, listed so a walk that read half the tree does
  not read like one that read all of it.

The two fatal kinds only fail the run when the *owning* package is one the committer can fix,
which is any of three things: a package in the checkout being scanned, one whose source is
elsewhere in the same workspace's `src`, or one below a `--fatal-under` prefix. Against a
package that is only a binary install they are reported as warnings.

Crossing package boundaries needs `AMENT_PREFIX_PATH`. Without it the walk stops at the
checkout's own launch files and says so - the manifest check still runs, since it never needed
a workspace.
"""

from pathlib import Path
from typing import NamedTuple, Optional
import argparse
import os
import sys

import lxml.etree as ET

from .helpers.condition_eval import evaluate_condition
from .helpers.exception_parser import parse_exceptions
from .helpers.find_launch_dependencies import (
    Walk,
    default_share_lookup,
    is_literal_package,
    scan_directories,
)
from .helpers.formatter.dependency_queries import (
    get_package_name,
    retrieve_exec_dependencies_with_conditions,
)
from .helpers.workspace import find_package_xml_files, find_workspace_root

__all__ = [
    "FATAL",
    "WARNING",
    "Finding",
    "Scope",
    "check",
    "main",
    "package_dirs_under",
]


#: Directories worth seeding from: `launch` and `components` as the manifest validator uses
#: them, plus the two that hold launch-manager component definitions.
DEFAULT_SEED_DIRS = (
    "launch",
    "components",
    "launch_manager_components",
    "launch_manager_configs",
)

#: How many unfollowable includes to list before summarising the rest as a count.
LISTED_UNREAD = 5

FATAL = "error"
WARNING = "warning"

UNDECLARED = "undeclared"
DEAD_INCLUDE = "dead-include"
NOT_INSTALLED = "not-installed"


class Finding(NamedTuple):
    """One thing worth saying about a launch tree."""

    kind: str
    severity: str
    package: str
    file: str
    """The launch file the reference was written in."""

    line: int
    owner: Optional[str] = None
    """The `package.xml` held responsible, where there is one."""

    launch_file: str = ""
    """For a dead include, the file the include named."""


def workspace_sources(paths: "list[str]") -> "list[str]":
    """The `<ws>/src` each path sits in, where there is one.

    A package is fixable when its source is anywhere in the same workspace, not only when it
    is in the repository being committed to - a workspace with one repository per package
    would otherwise put every sibling out of reach.
    """
    found: "list[str]" = []
    for one in paths:
        try:
            # Resolved first: find_workspace_root compares the path it is handed against an
            # absolute <ws>/src, and the hook passes `.`.
            root = find_workspace_root(Path(one).resolve())
        except ValueError:
            # Not a <ws>/src/<pkg> layout, so there is no workspace tier. Not a problem.
            continue
        source = root / "src"
        if source.is_dir() and str(source) not in found:
            found.append(str(source))
    return found


class Scope(NamedTuple):
    """Which manifests this run may fail on, and which names never to report at all."""

    source: dict
    """`package name -> package.xml`, for every package this run is allowed to fail on."""

    fatal_under: tuple = ()
    """The `--fatal-under` prefixes, kept so a manifest the index missed is still covered."""

    ignored: frozenset = frozenset()

    @classmethod
    def of(
        cls,
        paths: "list[str]",
        fatal_under: "tuple[str, ...]" = (),
        ignored: "frozenset[str]" = frozenset(),
    ) -> "Scope":
        """Index every package a finding may be fatal against.

        Three tiers: a listed path, such as an install space CI populates itself; the `src` of
        the surrounding workspace; and the checkout being scanned. Indexed least editable
        first, so where a package appears in more than one the most editable copy wins - that
        is both the manifest a finding is reported against and the one it is graded on.
        """
        index: dict = {}
        for root in (*fatal_under, *workspace_sources(paths), *paths):
            index.update(_index(find_package_xml_files([root])))
        return cls(index, fatal_under, ignored)

    def severity_of(self, manifest: Optional[str]) -> str:
        if manifest is None:
            return WARNING
        if manifest in self.source.values():
            return FATAL
        absolute = os.path.abspath(manifest)
        under = any(
            absolute.startswith(os.path.abspath(one) + os.sep)
            for one in self.fatal_under
        )
        return FATAL if under else WARNING


def workspace_is_available() -> bool:
    """Whether there is anything to resolve packages against."""
    return any(
        part for part in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    )


def package_dirs_under(paths: "list[str]") -> "list[str]":
    """The directories holding a `package.xml`, at any depth below `paths`.

    `check` works on one package at a time, but the hook is given a repository root. Reusing
    the validator's discovery means both entry points agree on what a package is, and on which
    directories (`COLCON_IGNORE` and friends) are skipped.
    """
    return sorted({os.path.dirname(one) for one in find_package_xml_files(paths)})


def check(
    package_dir: str,
    args: Optional[dict] = None,
    scope: Optional[Scope] = None,
    resolvable: Optional[bool] = None,
) -> "tuple[Walk, list[Finding]]":
    """Follow one package's launch tree and grade what it finds.

    `package_dir` is a single package directory, not a repository root - see
    :py:func:`package_dirs_under`. `scope` decides which findings are fatal, and defaults to
    treating this package alone as the checkout.
    """
    if scope is None:
        scope = Scope.of([package_dir])
    if resolvable is None:
        resolvable = workspace_is_available()

    seeds = [
        os.path.join(package_dir, name)
        for name in DEFAULT_SEED_DIRS
        if os.path.isdir(os.path.join(package_dir, name))
    ]

    # One walk over every seed directory, so a file reachable from two of them is read - and
    # reported - once.
    walk = scan_directories(seeds, args=args)

    return walk, _findings(walk, scope, resolvable)


# ── Reading the manifests a walk is graded against ───────────────────────────────────────

_MANIFESTS: "dict[str, tuple[Optional[str], frozenset, frozenset]]" = {}


def _read(manifest: str) -> "tuple[Optional[str], frozenset, frozenset]":
    """`(package name, declared exec deps, names to ignore)`, parsed once per path.

    Exec dependencies, because a launch file names what it runs. Conditions are evaluated and
    `<!-- validator:ignore ... -->` is honoured, so this agrees with what
    `package-xml-validator` would say about the same manifest.
    """
    if manifest not in _MANIFESTS:
        try:
            parser = ET.XMLParser(no_network=True, resolve_entities=False)
            root = ET.parse(manifest, parser).getroot()
        except (OSError, ET.XMLSyntaxError):
            _MANIFESTS[manifest] = (None, frozenset(), frozenset())
        else:
            declared = frozenset(
                name
                for name, condition in retrieve_exec_dependencies_with_conditions(root)
                if evaluate_condition(condition)
            )
            _MANIFESTS[manifest] = (
                get_package_name(root),
                declared,
                parse_exceptions(root).ignored_deps,
            )
    return _MANIFESTS[manifest]


def _index(manifests: "list[str]") -> dict:
    """`package name -> package.xml`, for looking a source manifest up by name."""
    found = {}
    for manifest in manifests:
        name = _read(manifest)[0]
        if name:
            found[name] = manifest
    return found


def _owner(path: str, source: dict) -> Optional[str]:
    """The `package.xml` responsible for `path`, preferring the checkout to an installed copy.

    Includes resolve through `find-pkg-share`, so most files a walk reads are installed copies.
    Where the same package is also in the checkout, that is the manifest a committer can edit
    and the one the run should be graded against.
    """
    directory = os.path.dirname(os.path.abspath(path))
    while True:
        manifest = os.path.join(directory, "package.xml")
        if os.path.isfile(manifest):
            name = _read(manifest)[0]
            return source.get(name, manifest) if name else manifest
        parent = os.path.dirname(directory)
        if parent == directory:
            return None
        directory = parent


# ── Grading a walk ───────────────────────────────────────────────────────────────────────


def _findings(walk: Walk, scope: Scope, resolvable: bool) -> "list[Finding]":
    owners = {one: _owner(one, scope.source) for one in walk.visited}

    # Absent as far as the walk is concerned. An include into one of these could not be
    # followed, so calling it a package that fails to ship a file would be a guess.
    absent = (
        {one for one in walk.packages if default_share_lookup(one) is None}
        if resolvable
        else set()
    )

    findings: list[Finding] = []
    findings.extend(_undeclared(walk, scope, owners))
    if resolvable:
        findings.extend(_dead(walk, scope, owners, absent))
        findings.extend(_absent(walk, scope, owners, absent))

    return sorted(
        findings, key=lambda one: (one.severity != FATAL, one.kind, one.package)
    )


def _muted(package: str, scope: Scope, manifest: Optional[str]) -> bool:
    """Whether this package was asked to be left out, globally or by the manifest citing it."""
    if package in scope.ignored:
        return True
    return manifest is not None and package in _read(manifest)[2]


def _first_mention(walk: Walk) -> dict:
    """Where each package was first named, so a finding can cite one place rather than all."""
    where: dict = {}
    for one in walk.references:
        where.setdefault(one.package, one)
    return where


def _undeclared(walk: Walk, scope: Scope, owners: dict) -> "list[Finding]":
    """References a launch file makes that its own package.xml does not declare."""
    findings = []
    seen = set()

    for one in walk.references:
        manifest = owners.get(one.file)
        if manifest is None or _muted(one.package, scope, manifest):
            continue

        name, declared, _ = _read(manifest)
        if one.package == name or one.package in declared:
            continue

        if (manifest, one.package) in seen:
            continue
        seen.add((manifest, one.package))

        findings.append(
            Finding(
                UNDECLARED,
                scope.severity_of(manifest),
                one.package,
                one.file,
                one.line,
                manifest,
            )
        )

    return findings


def _dead(walk: Walk, scope: Scope, owners: dict, absent: set) -> "list[Finding]":
    """Includes whose package is here and whose file is not.

    An include into a package that is not here at all could not be followed for a different
    reason, and one whose name still holds a substitution names no package at all - neither is
    evidence that a file is missing.
    """
    return [
        Finding(
            DEAD_INCLUDE,
            scope.severity_of(owners.get(one.file)),
            one.package,
            one.file,
            one.line,
            owners.get(one.file),
            one.launch_file,
        )
        for one in walk.unresolved
        if is_literal_package(one.package)
        and one.package not in absent
        and not _muted(one.package, scope, owners.get(one.file))
    ]


def _absent(walk: Walk, scope: Scope, owners: dict, absent: set) -> "list[Finding]":
    """Packages nothing in this workspace provides.

    A package that is in the checkout is left out: it is not missing, only unbuilt, which is
    the ordinary state of a CI runner and says nothing about the tree.
    """
    return [
        Finding(
            NOT_INSTALLED, WARNING, package, one.file, one.line, owners.get(one.file)
        )
        for package, one in _first_mention(walk).items()
        if package in absent
        and package not in scope.source
        and not _muted(package, scope, owners.get(one.file))
    ]


# ── Saying it ────────────────────────────────────────────────────────────────────────────


def _where(path: str, package_dir: str) -> str:
    """A path a reader can act on.

    Relative while the file belongs to this package, absolute once it does not - a chain of
    `../..` out of the workspace and into `/opt` tells nobody anything.
    """
    relative = os.path.relpath(path, package_dir)
    return os.path.abspath(path) if relative.startswith("..") else relative


#: Indent for the detail lines under a finding, and the width their labels are padded to.
DETAIL = " " * 12
LABEL = 9


def _owner_name(manifest: Optional[str]) -> str:
    """What to call the package a finding is charged to.

    The manifest path alone makes a reader work out which package is at fault from a
    `share/<name>/package.xml` fragment, so the name leads and the path follows it.
    """
    if manifest is None:
        return "its own package"
    return _read(manifest)[0] or os.path.basename(os.path.dirname(manifest))


def _report(
    package_dir: str,
    walk: Walk,
    findings: "list[Finding]",
    scope: Scope,
    resolvable: bool,
) -> None:
    """Print what this package's tree turned up, or nothing at all if it turned up nothing."""
    unread = _unfollowable(walk, findings)
    if not findings and not unread:
        return

    reached = {_owner(one, scope.source) for one in walk.visited}
    reached.discard(None)

    print(
        f"{os.path.basename(os.path.abspath(package_dir))}: followed this package's launch "
        f"tree - {len(walk.visited)} launch file(s) across {len(reached)} package(s), naming "
        f"{len(walk.packages)} package(s)."
    )

    # A package that is both undeclared and absent is one problem, so the absence is a clause
    # on the manifest finding rather than a second line under it.
    uninstalled = {one.package for one in findings if one.kind == NOT_INSTALLED}
    also_undeclared = {one.package for one in findings if one.kind == UNDECLARED}

    for one in findings:
        if one.kind == NOT_INSTALLED and one.package in also_undeclared:
            continue
        print()
        print(f"  {one.severity:<7} {_headline(one, uninstalled)}")
        if one.kind == UNDECLARED and one.owner:
            print(f"{DETAIL}{'manifest':<{LABEL}}{_where(one.owner, package_dir)}")
        print(
            f"{DETAIL}{'named in':<{LABEL}}{_where(one.file, package_dir)}:{one.line}"
        )

    if unread:
        print()
        print(f"  {WARNING:<7} {len(unread)} include(s) could not be followed:")
        for edge in unread[:LISTED_UNREAD]:
            target = f"{edge.package or '(computed)'}/{edge.launch_file}"
            print(f"{DETAIL}{_where(edge.file, package_dir)}:{edge.line}  {target}")
        if len(unread) > LISTED_UNREAD:
            print(f"{DETAIL}... and {len(unread) - LISTED_UNREAD} more")

    if not resolvable:
        print()
        print(
            f"  {WARNING:<7} AMENT_PREFIX_PATH is empty, so includes into other packages were "
            "not followed."
        )


def _headline(one: Finding, uninstalled: set) -> str:
    if one.kind == NOT_INSTALLED:
        return f"{one.package} is not installed in this workspace"

    if one.kind == DEAD_INCLUDE:
        return f"{one.package} does not ship {one.launch_file}"

    absent = " (and is not installed here)" if one.package in uninstalled else ""
    return f"{one.package} is not declared by {_owner_name(one.owner)}{absent}"


def _unfollowable(walk: Walk, findings: "list[Finding]") -> list:
    """Includes no finding above has already accounted for."""
    explained = {
        one.package for one in findings if one.kind in (DEAD_INCLUDE, NOT_INSTALLED)
    }
    return [one for one in walk.unresolved if one.package not in explained]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Check that everything a package's launch tree references exists here, "
        "and that every manifest along the way declares what its launch files name."
    )
    parser.add_argument(
        "src",
        nargs="*",
        default=["."],
        help="Files or directories to search for packages. Defaults to the current directory.",
    )
    parser.add_argument(
        "--fatal-under",
        action="append",
        default=[],
        metavar="PATH",
        help="Also treat packages below PATH as yours to fix, so findings against them fail "
        "the run. Repeat for several. Packages in SRC, and packages whose source is elsewhere "
        "in the same workspace, already count without this.",
    )
    parser.add_argument(
        "--ignore",
        action="append",
        default=[],
        metavar="NAME",
        help="A package to leave out of every report. Repeat for several.",
    )
    parser.add_argument(
        "--warn-only",
        action="store_true",
        help="Report everything but always exit zero, for introducing the check to a "
        "repository that already has findings.",
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

    args = {}
    for item in parsed.arg:
        name, _, value = item.partition("=")
        if name:
            args[name] = value

    manifests = find_package_xml_files(parsed.src)
    if not manifests:
        print("check-launch-tree: no packages found. Nothing to check.")
        return

    scope = Scope.of(parsed.src, tuple(parsed.fatal_under), frozenset(parsed.ignore))
    resolvable = workspace_is_available()

    fatal = 0
    for package_dir in sorted({os.path.dirname(one) for one in manifests}):
        walk, findings = check(package_dir, args or None, scope, resolvable)
        _report(package_dir, walk, findings, scope, resolvable)
        fatal += sum(1 for one in findings if one.severity == FATAL)

    if fatal and not parsed.warn_only:
        sys.exit(1)


if __name__ == "__main__":
    main()
