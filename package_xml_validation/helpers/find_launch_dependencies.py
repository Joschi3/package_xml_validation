#!/usr/bin/env python3
"""
find_launch_dependencies.py

Recursively search a ROS 2 package's launch/ folder and extract
all referenced ROS 2 package names via a small set of regexes.

Ignores matches that occur inside comments:
- Python: # line comments, and triple-quoted blocks
- XML/.launch: <!-- ... --> comments
- YAML: # line comments

Each pattern also declares which file formats it applies to, because a pattern is a piece of
syntax and the same characters mean different things in different languages.
"""

import ast
import glob
import logging
import os
import re
from typing import Callable, NamedTuple, Optional

import yaml

logger = logging.getLogger(__name__)


# Which file formats a pattern is allowed to match.
FORMAT_YAML = "yaml"
FORMAT_XML = "xml"
FORMAT_PY = "py"
FORMAT_TOML = "toml"


#: For patterns whose syntax means the same thing in every format, such as
#: `$(find-pkg-share x)`.
ANY_FORMAT = frozenset({FORMAT_YAML, FORMAT_XML, FORMAT_PY, FORMAT_TOML})

#: `(regex, formats)`. The formats are the point: see the note above.
PATTERNS = [
    # YAML-style:  pkg: <pkg_name>
    #
    # Anchored to the start of a line, allowing the leading `- ` of a sequence item. Without
    # the anchor any key *ending* in `pkg` matches - `mypkg: foo` yielded a package named
    # `foo`. Not applied to Python, where `pkg: str` is a type annotation and used to yield a
    # package named `str`, nor to a string literal that happens to contain `pkg:`.
    (r"(?m)^[\s\-]*pkg\s*:\s*['\"]?([A-Za-z0-9_]+)['\"]?", {FORMAT_YAML, FORMAT_XML}),
    # Hector launch component:  package: <pkg_name>
    (
        r"(?m)^[\s\-]*package\s*:\s*['\"]?([A-Za-z0-9_]+)['\"]?",
        {FORMAT_YAML},
    ),
    # Python Node-family constructors: Node / LifecycleNode / ComposableNode /
    # ComposableLifecycleNode (..., package='<pkg_name>', ...)
    (
        r"(?:^|[^\w])(?:Node|LifecycleNode|ComposableNode|ComposableLifecycleNode)\s*\(\s*[^)]*?\bpackage\s*=\s*['\"]([A-Za-z0-9_]+)['\"]",
        {FORMAT_PY},
    ),
    # XML node tag: <node pkg="foo" ...>
    (r"<node[^>]*?\bpkg\s*=\s*['\"]?([A-Za-z0-9_]+)['\"]?", {FORMAT_XML}),
    # XML composable node: <composable_node pkg="foo" ...>
    (r"<composable_node[^>]*?\bpkg\s*=\s*['\"]?([A-Za-z0-9_]+)['\"]?", {FORMAT_XML}),
    # XML node container: <node_container pkg="foo" ...>
    (r"<node_container[^>]*?\bpkg\s*=\s*['\"]?([A-Za-z0-9_]+)['\"]?", {FORMAT_XML}),
    # get_package_share_directory('foo')
    (r"get_package_share_directory\(\s*['\"]([A-Za-z0-9_]+)['\"]\s*\)", {FORMAT_PY}),
    # get_package_share_path('foo')
    (r"get_package_share_path\(\s*['\"]([A-Za-z0-9_]+)['\"]\s*\)", {FORMAT_PY}),
    # get_package_prefix('foo')
    (r"get_package_prefix\(\s*['\"]([A-Za-z0-9_]+)['\"]\s*\)", {FORMAT_PY}),
    # FindPackageShare('foo')
    (r"FindPackageShare\(\s*['\"]([A-Za-z0-9_]+)['\"]\s*\)", {FORMAT_PY}),
    # FindPackageShare(package='foo')
    (r"FindPackageShare\(\s*package\s*=\s*['\"]([A-Za-z0-9_]+)['\"]\s*\)", {FORMAT_PY}),
    # FindPackagePrefix('foo') / FindPackagePrefix(package='foo')
    (
        r"FindPackagePrefix\(\s*(?:package\s*=\s*)?['\"]([A-Za-z0-9_]+)['\"]\s*\)",
        {FORMAT_PY},
    ),
    # $(find-pkg-share foo)
    (r"\$\(\s*find-pkg-share\s+([A-Za-z0-9_]+)\s*\)", ANY_FORMAT),
    # $(find-pkg-prefix foo)
    (r"\$\(\s*find-pkg-prefix\s+([A-Za-z0-9_]+)\s*\)", ANY_FORMAT),
    # better_launch Python:  bl.node("pkg", ...)  /  bl.include("pkg", ...)
    (r"\bbl\.node\(\s*['\"]([A-Za-z0-9_]+)['\"]", {FORMAT_PY}),
    (r"\bbl\.include\(\s*['\"]([A-Za-z0-9_]+)['\"]", {FORMAT_PY}),
    # better_launch TOML:  package = "pkg"
    (r"(?m)^\s*package\s*=\s*['\"]([A-Za-z0-9_]+)['\"]", {FORMAT_TOML}),
]

COMPILED_PATTERNS = [(re.compile(rx), formats) for rx, formats in PATTERNS]

#: Kept for anything importing the old names. `COMPILED` no longer decides what is applied to
#: a file - :py:func:`_patterns_for` does, by format.
REGEX_EXPR = [rx for rx, _ in PATTERNS]
COMPILED = [rx for rx, _ in COMPILED_PATTERNS]
TOML_REGEX_EXPR = [rx for rx, formats in PATTERNS if formats == {FORMAT_TOML}]
TOML_COMPILED = [re.compile(rx) for rx in TOML_REGEX_EXPR]


def _format_of(path: str) -> Optional[str]:
    """Which syntax a file is written in, or None if we do not scan it."""
    s = path.lower()
    if s.endswith(".py"):
        return FORMAT_PY
    if s.endswith(".xml") or s.endswith(".launch"):
        return FORMAT_XML
    if s.endswith((".yaml", ".yml")):
        return FORMAT_YAML
    if s.endswith(".toml"):
        return FORMAT_TOML
    return None


def _patterns_for(fmt: str) -> list[tuple["re.Pattern[str]", int]]:
    """The compiled patterns that mean anything in `fmt`."""
    return [
        (rx, i) for i, (rx, formats) in enumerate(COMPILED_PATTERNS) if fmt in formats
    ]


_TRIPLE_QUOTE_BLOCK = re.compile(r"(?s)(['\"]{3})(?:.*?)(\1)")
_XML_COMMENT_BLOCK = re.compile(r"(?s)<!--.*?-->")


def _blank_out(pattern: "re.Pattern[str]", text: str) -> str:
    """Remove a block but keep the lines it spanned.

    Deleting a multi-line comment outright shifts every line after it, which is invisible
    while matches are only collected as names and wrong the moment one is reported with a line
    number. The newlines are kept so a match's position still means what it says.
    """
    return pattern.sub(lambda m: "\n" * m.group(0).count("\n"), text)


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
    # 1) drop triple-quoted blocks, keeping the lines they spanned
    text = _blank_out(_TRIPLE_QUOTE_BLOCK, text)
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
    return _blank_out(_XML_COMMENT_BLOCK, text)


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
    fmt = _format_of(path)
    if fmt is None:
        return

    # A TOML file is only a launch file when it is under a launch/ directory - `package = "x"`
    # is far too common a line in ordinary project TOML to read as a dependency anywhere else.
    if fmt == FORMAT_TOML and not _is_under_launch_dir(path):
        return

    with open(path, encoding="utf-8") as f:
        text = f.read()

    text = _decomment_for_suffix(path, text)

    for rx, index in _patterns_for(fmt):
        for m in rx.finditer(text):
            pkg = m.group(1)
            found.add(pkg)
            if verbose:
                logger.debug(
                    "Found package '%s' in %s with regex %s",
                    pkg,
                    os.path.basename(path),
                    REGEX_EXPR[index],
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


# ── Following a launch tree from a seed ──────────────────────────────────────────────────
#
# `scan_files` covers every launch file a package ships, which is the manifest question.
# Seeding from one launch file asks instead what a particular system launches: it descends
# into other packages' launch files, and ignores files the seed does not reach.
#
# The walk follows include *edges* only. A package named by `pkg:` on a node is a leaf - there
# is no launch file behind it, and scanning everything that package ships would report launch
# files nobody runs.


class Reference(NamedTuple):
    """One package name, and where it was written."""

    package: str
    file: str
    line: int


class Edge(NamedTuple):
    """An include: one launch file naming another, and whether we could find it."""

    package: str
    launch_file: str
    file: str
    line: int
    resolved: Optional[str]

    by_basename: bool = False
    """Whether `launch_file` is a bare filename to search the share directory for.

    True only for launch-manager components, which name a file and leave the manager to find
    it. Everywhere else the path is written out and has to resolve exactly.
    """


class Walk(NamedTuple):
    """What following a seed found.

    A launch tree is only partly made of includes, so parts of it are outside this scan:

    * a node that reads a config and starts processes of its own is opaque;
    * `if:` / `unless:` branches are collected as a union, not as the set one run would take;
    * anything assembled at runtime (`$(eval ...)`, `$(command ...)`) is left unresolved.
    """

    references: list[Reference]
    """Every package named anywhere in the tree, with provenance. May repeat a package."""

    edges: list[Edge]
    """Every include encountered, resolved or not."""

    visited: list[str]
    """The launch files actually read, seed first."""

    @property
    def packages(self) -> list[str]:
        return sorted({one.package for one in self.references})

    @property
    def unresolved(self) -> list[Edge]:
        """Includes this could not follow - a package that is not installed, a file that is
        not where it should be, or a name built from a substitution.

        Empty means nothing this scan can see was left unfollowed, which is not the same as
        the tree being fully explored. See the class docstring.
        """
        return [one for one in self.edges if one.resolved is None]


#: `$(find-pkg-share x)/some/path` - the path is kept so an include can be resolved from it.
#: The package part is not restricted to an identifier, because a launch file may build it
#: from a substitution: `$(find-pkg-share $(var robot)_sim)/launch/spawn.launch.py`. Such a
#: name is reported unresolved rather than skipped. One level of nesting covers every form
#: seen in practice.
_SHARE_PATH = re.compile(
    r"\$\(\s*find-pkg-share\s+((?:[^()\s]|\([^()]*\))+)\s*\)/([A-Za-z0-9_./\-]+)"
)

#: A package name we could look up, as opposed to one assembled at launch time.
_LITERAL_PACKAGE = re.compile(r"^[A-Za-z0-9_]+$")

_LAUNCH_SUFFIXES = (".launch.py", ".launch.xml", ".launch.yaml", ".launch.yml")


def _find_in_share(
    share: str, relative: str, by_basename: bool = False
) -> Optional[str]:
    """The launch file inside a package's share directory, or None.

    The written path, and for a bare filename also the same basename anywhere under the
    package - a launch-manager component says `nav2.launch.yaml` and lets the manager find it
    at `launch/navigation/nav2.launch.yaml`. First match in sorted order if it occurs twice.

    Searching for a written-out path instead would resolve a renamed or uninstalled launch
    file to a same-named one elsewhere in the package, and report what *that* file names.
    """
    direct = os.path.join(share, relative)
    if os.path.isfile(direct):
        return direct

    if not by_basename:
        return None

    matches = sorted(
        glob.glob(os.path.join(share, "**", os.path.basename(relative)), recursive=True)
    )
    return matches[0] if matches else None


def _line_of(text: str, position: int) -> int:
    return text.count("\n", 0, position) + 1


def _looks_like_a_launch_file(path: str) -> bool:
    """A path under `launch/`, or named `*.launch.*`.

    Neither test alone is enough: a launch directory also holds files that are not launch
    files, and `$(find-pkg-share x)/config/params.yaml` has an extension we scan.
    """
    lowered = path.lower()
    return lowered.endswith(_LAUNCH_SUFFIXES) or "/launch/" in f"/{lowered}"


def default_share_lookup(package: str) -> Optional[str]:
    """Where a package's share directory is, read from AMENT_PREFIX_PATH.

    Not `ament_index_python`, so the library works without ROS on the path. Pass your own
    lookup to `scan_from` to resolve against something else.
    """
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        share = os.path.join(prefix, "share", package)
        if os.path.isdir(share):
            return share
    return None


def scan_file_detailed(path: str, scope: Optional[dict] = None) -> list[Reference]:
    """Every package `path` names, each with the line it was named on.

    The same matches `scan_file` collects, keeping their provenance.

    `scope` expands `$(var x)` first, so a package whose name is assembled from an argument is
    reported under the name it will actually have. Substitutions never span lines, so the line
    numbers survive the expansion.
    """
    fmt = _format_of(path)
    if fmt is None:
        return []

    if fmt == FORMAT_TOML and not _is_under_launch_dir(path):
        return []

    try:
        with open(path, encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return []

    text = _decomment_for_suffix(path, text)
    if scope:
        text = expand_vars(text, scope)

    found: list[Reference] = []
    for rx, _ in _patterns_for(fmt):
        for m in rx.finditer(text):
            found.append(Reference(m.group(1), path, _line_of(text, m.start())))

    return found


def _edges_in(path: str, scope: Optional[dict] = None) -> list[Edge]:
    """The includes `path` makes, before any attempt to resolve them.

    Three kinds: a frontend `$(find-pkg-share ...)` substitution, a launch-manager component
    naming a package and a file, and a Python `IncludeLaunchDescription`.
    """
    fmt = _format_of(path)
    if fmt is None:
        return []

    edges: list[Edge] = []

    if fmt == FORMAT_PY:
        edges.extend(_python_edges(path))

    try:
        with open(path, encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return edges

    text = _decomment_for_suffix(path, text)
    if scope:
        text = expand_vars(text, scope)

    for m in _SHARE_PATH.finditer(text):
        package, relative = m.group(1), m.group(2)
        if _looks_like_a_launch_file(relative):
            # A computed name is kept as written; `_walk` leaves it unresolved.
            edges.append(Edge(package, relative, path, _line_of(text, m.start()), None))

    if fmt == FORMAT_YAML:
        edges.extend(_component_edges(path, text))

    return edges


def _component_edges(path: str, text: str) -> list[Edge]:
    """Launch-manager components in a YAML file: a mapping with `package` and `launch_file`.

    Parsed rather than pattern-matched, so that the two keys have to belong to the *same*
    mapping. A regex scanning a window of following lines paired a node-only component's
    `package` with the next component's `launch_file`, which both invented a finding and left
    the real include unfollowed.
    """
    try:
        root = yaml.compose(text)
    except yaml.YAMLError:
        return []

    edges: list[Edge] = []

    def visit(node: "yaml.Node") -> None:
        if isinstance(node, yaml.SequenceNode):
            for item in node.value:
                visit(item)
            return
        if not isinstance(node, yaml.MappingNode):
            return

        scalars = {
            key.value: (value, key)
            for key, value in node.value
            if isinstance(key, yaml.ScalarNode) and isinstance(value, yaml.ScalarNode)
        }
        package, launch_file = scalars.get("package"), scalars.get("launch_file")
        if package and launch_file:
            edges.append(
                Edge(
                    package[0].value,
                    os.path.join("launch", launch_file[0].value),
                    path,
                    package[1].start_mark.line + 1,
                    None,
                    by_basename=True,
                )
            )

        for _, value in node.value:
            visit(value)

    visit(root)
    return edges


# ── Resolving what a launch file leaves to be filled in later ────────────────────────────

#: `$(var name)`. Other substitution types - `$(env x)`, `$(eval ...)`, `$(command ...)` -
#: depend on the environment or on running code and are left unexpanded.
_VAR = re.compile(r"\$\(\s*var\s+([A-Za-z0-9_]+)\s*\)")


def declared_args(path: str) -> dict:
    """The `{name: default}` a YAML frontend launch file declares.

    Only defaults that are written down. An argument with no default is one the caller has to
    supply, and guessing one would resolve an include to a file this system never loads.
    """
    if _format_of(path) != FORMAT_YAML:
        return {}

    try:
        with open(path, encoding="utf-8") as f:
            document = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        # Unparsable: nothing to expand from, so names stay as written and end up unresolved.
        return {}

    if not isinstance(document, dict):
        return {}

    found = {}
    for entry in document.get("launch") or []:
        if not isinstance(entry, dict):
            continue
        arg = entry.get("arg")
        if isinstance(arg, dict) and "name" in arg and "default" in arg:
            found[str(arg["name"])] = str(arg["default"])

    return found


def expand_vars(text: str, scope: dict, rounds: int = 10) -> str:
    """Replace `$(var x)` from `scope`, repeatedly, leaving unknown names as written.

    Repeatedly because a default may cite another argument (`robot_name: "$(var robot)"`).
    Bounded rather than run to a fixpoint, so mutually defined arguments cannot hang a scan.
    """
    for _ in range(rounds):
        expanded = _VAR.sub(lambda m: scope.get(m.group(1), m.group(0)), text)
        if expanded == text:
            return expanded
        text = expanded

    return text


def _scope_for(path: str, args: Optional[dict]) -> dict:
    """What `$(var x)` means in this file: its own declared defaults, then the caller's values.

    The caller's win, so a caller that knows what it is about to launch with follows the tree
    that will come up rather than the one the defaults describe.
    """
    scope = declared_args(path)
    if args:
        scope.update({str(k): str(v) for k, v in args.items()})

    # A default may name another argument, so the table has to be expanded against itself.
    return {name: expand_vars(value, scope) for name, value in scope.items()}


# ── Includes written in Python ───────────────────────────────────────────────────────────

#: Calls that name a package's share directory. Everything else is reported as an include
#: this scan could not follow - see `_python_edges`.
_SHARE_CALLS = (
    "FindPackageShare",
    "get_package_share_directory",
    "get_package_share_path",
)


def _called_name(node: ast.Call) -> str:
    """The trailing name of a call target: `f(...)` and `a.b.f(...)` both give `f`.

    Qualified calls are common in launch files - `launch.actions.IncludeLaunchDescription`,
    `ament_index_python.get_package_share_directory` - and matching only `ast.Name` made them
    invisible: no edge at all, rather than one reported as unreadable.
    """
    func = node.func
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return ""


def _string_of(node: ast.AST) -> Optional[str]:
    return (
        node.value
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
        else None
    )


def _path_expression(node: ast.AST, bound: dict) -> "Optional[tuple[str, list[str]]]":
    """`(package, path segments)` for an expression that names a file inside a package.

    Handles the three ways one is written, and their combinations:

        get_package_share_directory("x")            -> ("x", [])
        os.path.join(<expr>, "launch", "a.py")      -> the expr's package, its parts + these
        PathJoinSubstitution([<expr>, "launch"])    -> the same

    `bound` carries names already resolved, so a chain of assignments works.

    Returns None as soon as anything is not a literal or a known name. Control flow is not
    followed: a name assigned twice resolves to the last assignment seen, so the result is
    only used to find more launch files to read, never to claim one runs.
    """
    if isinstance(node, ast.Name):
        return bound.get(node.id)

    if not isinstance(node, ast.Call):
        return None

    called = _called_name(node)

    if called in _SHARE_CALLS:
        named = _string_of(node.args[0]) if node.args else None
        return (named, []) if named else None

    items: "list[ast.AST]" = []
    if called == "join":
        items = list(node.args)
    elif called == "PathJoinSubstitution":
        if node.args and isinstance(node.args[0], (ast.List, ast.Tuple)):
            items = list(node.args[0].elts)

    if not items:
        return None

    package: Optional[str] = None
    parts: list[str] = []

    for item in items:
        literal = _string_of(item)
        if literal is not None:
            parts.append(literal)
            continue

        resolved = _path_expression(item, bound)
        if resolved is None:
            return None
        if package is not None:
            # Two package roots in one path is not something to guess at.
            return None
        package, parts = resolved[0], list(resolved[1]) + parts

    return (package, parts) if package else None


def _share_bindings(tree: ast.Module, rounds: int = 3) -> dict:
    """Every name bound to a location inside a package, anywhere in the file.

    Anywhere rather than at module level, since the idiom is as often a local inside
    `generate_launch_description`. Repeated because one binding may be built from another and
    `ast.walk` does not visit them in dependency order.
    """
    bound: dict = {}

    for _ in range(rounds):
        before = len(bound)

        for node in ast.walk(tree):
            if not isinstance(node, ast.Assign) or len(node.targets) != 1:
                continue
            target = node.targets[0]
            if not isinstance(target, ast.Name):
                continue

            resolved = _path_expression(node.value, bound)
            if resolved is not None:
                bound[target.id] = resolved

        if len(bound) == before:
            break

    return bound


def _python_edges(path: str) -> "list[Edge]":
    """Every `IncludeLaunchDescription` in a Python launch file.

    Resolved where the package and path are literals. Everything else is still reported, with
    its line, as an include this scan could not read.
    """
    try:
        with open(path, encoding="utf-8") as f:
            tree = ast.parse(f.read())
    except (OSError, SyntaxError):
        return []

    edges: list[Edge] = []
    bound = _share_bindings(tree)

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if _called_name(node) != "IncludeLaunchDescription":
            continue

        package: Optional[str] = None
        parts: list[str] = []
        for inner in ast.walk(node):
            resolved = (
                _path_expression(inner, bound) if isinstance(inner, ast.Call) else None
            )
            if resolved is not None and resolved[1]:
                package, parts = resolved
                break

        if package and parts:
            edges.append(Edge(package, os.path.join(*parts), path, node.lineno, None))
        else:
            edges.append(
                Edge(
                    "",
                    "an IncludeLaunchDescription this scan could not read",
                    path,
                    node.lineno,
                    None,
                )
            )

    return edges


def scan_directory(
    directory: str,
    share_lookup: Callable[[str], Optional[str]] = default_share_lookup,
    args: Optional[dict] = None,
    max_depth: int = 12,
) -> Walk:
    """Every launch file in `directory`, followed - one walk, not one per file.

    The recursive counterpart to :py:func:`scan_files`: that reads what a package ships, this
    also reads what those files pull in. Sibling launch files tend to include the same things,
    so a single shared walk reads each of them once.

    Args:
        directory: Scanned recursively for launch files to use as seeds.
        share_lookup: `package -> share directory or None`.
        args: Values for `$(var x)`, as in :py:func:`scan_from`.
        max_depth: How far to follow includes below each seed.

    Returns:
        A :py:class:`Walk` over all of them.

    """
    seeds = []

    for root, _, files in os.walk(directory):
        for name in sorted(files):
            full = os.path.join(root, name)
            if _format_of(full) is None:
                continue
            if _format_of(full) == FORMAT_TOML and not _is_under_launch_dir(full):
                continue
            seeds.append(full)

    return _walk(seeds, share_lookup, args, max_depth)


def scan_from(
    seed: str,
    share_lookup: Callable[[str], Optional[str]] = default_share_lookup,
    args: Optional[dict] = None,
    max_depth: int = 12,
) -> Walk:
    """Follow a launch file's includes, collecting every package the tree names.

    Args:
        seed: The launch file to start from.
        share_lookup: `package -> share directory or None`. Defaults to AMENT_PREFIX_PATH.
        args: Values for `$(var x)`, overriding any default the file declares. Pass what the
            system will be launched with to follow the tree that will come up; omit them to
            follow the one the defaults describe. Applied at any depth.
        max_depth: How far to follow includes. A guard against runaway recursion.

    Returns:
        A :py:class:`Walk`. Includes that could not be resolved are in `unresolved` rather
        than being guessed at or dropped, so a caller reporting coverage knows how much of
        the tree went unread.

    """
    return _walk([seed], share_lookup, args, max_depth)


def _walk(
    seeds: "list[str]",
    share_lookup: Callable[[str], Optional[str]],
    args: Optional[dict],
    max_depth: int,
) -> Walk:
    """The walk itself, over one seed or many.

    The visited set spans every seed, so a file two seeds both reach is read - and reported -
    once.
    """
    references: list[Reference] = []
    edges: list[Edge] = []
    visited: list[str] = []
    seen: set[str] = set()

    queue = [(os.path.abspath(one), 0) for one in seeds]

    while queue:
        current, depth = queue.pop(0)

        if current in seen or depth > max_depth:
            continue
        seen.add(current)

        if not os.path.isfile(current):
            continue

        visited.append(current)

        # Each file expands with its own declared defaults under the caller's values, so a
        # child that declares an argument the parent never mentions still resolves.
        scope = _scope_for(current, args)

        references.extend(scan_file_detailed(current, scope))

        for edge in _edges_in(current, scope):
            if not _LITERAL_PACKAGE.match(edge.package):
                # Built from a substitution, so there is nothing to look up.
                edges.append(edge)
                continue

            share = share_lookup(edge.package)
            target = (
                _find_in_share(share, edge.launch_file, edge.by_basename)
                if share
                else None
            )

            if target is not None and os.path.isfile(target):
                edges.append(edge._replace(resolved=target))
                queue.append((target, depth + 1))
            else:
                edges.append(edge)

    return Walk(references=references, edges=edges, visited=visited)
