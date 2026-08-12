"""Which packages each host runs, derived from a launch-manager configuration.

A launch-manager repository states twice, in two machine-readable places, what a host runs:
`launch_manager_configs/*.yaml` assigns a component to a host, and the component's own definition
names the launch file it starts. Following that launch file gives every package the host needs.

That makes a per-host dependency set derivable rather than hand-maintained, which matters when the
launch files live in one package and the dependencies are declared in a sibling *install set* per
host - the shape a workspace takes when some of its machines are too small to install everything.

The result is a **lower bound**. A controller plugin named only by class in a parameter file, or a
package passed as an argument to a generic component, is named nowhere this can reach. Callers may
add what is derived; they must never remove what is not.
"""

from __future__ import annotations

import glob
import os
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import lxml.etree as ET
import yaml

from .find_launch_dependencies import default_share_lookup, scan_from
from .formatter.dependency_queries import retrieve_exec_dependencies

if TYPE_CHECKING:
    from .package_types import XmlElement

#: Where a repository keeps the files this reads.
CONFIG_DIR = "launch_manager_configs"
COMPONENT_DIR = "launch_manager_components"

#: The `<export>` child by which a package claims to be a host's install set.
HOST_EXPORT = "launch_manager_host"

#: The package that ships components shared between robots. Components a configuration names but
#: the repository does not define are looked up here.
SHARED_COMPONENTS = "launch_manager_common_components"


@dataclass
class HostDependencies:
    """What each host runs, and how much of the question went unanswered."""

    packages: dict[str, set[str]] = field(default_factory=dict)
    """`host -> packages its components need`."""

    unresolved: list[str] = field(default_factory=list)
    """Components named by a configuration whose definition was not found."""

    problems: list[str] = field(default_factory=list)
    """Why a derivation is missing or incomplete, in words a reader can act on."""

    @property
    def complete(self) -> bool:
        """Whether every component was read.

        False means the sets below are short by an unknown amount, so they may be reported but
        must not be filled in - a partial derivation that looks complete is worse than none.
        """
        return not self.unresolved and not self.problems


def declared_host(root: XmlElement) -> str | None:
    """The host a package claims to be the install set for, or None.

    `<export><launch_manager_host>athena-gripper</launch_manager_host></export>`. The export
    section is the sanctioned place for a tool's own metadata: the schema check only inspects
    top-level children, and the build-type step leaves other export children alone.
    """
    export = root.find("export")
    if export is None:
        return None
    element = export.find(HOST_EXPORT)
    if element is None or not element.text:
        return None
    return element.text.strip() or None


def find_config_package(package_dirs: list[str]) -> str | None:
    """The package holding the launch-manager configuration, if exactly one does."""
    holders = [
        one for one in package_dirs if os.path.isdir(os.path.join(one, CONFIG_DIR))
    ]
    return holders[0] if len(holders) == 1 else None


def derive(package_dirs: list[str]) -> HostDependencies:
    """`host -> packages` for the launch-manager configuration found under `package_dirs`."""
    holders = [
        one for one in package_dirs if os.path.isdir(os.path.join(one, CONFIG_DIR))
    ]

    if not holders:
        return HostDependencies(
            problems=[f"no package holds a {CONFIG_DIR}/ directory"]
        )
    if len(holders) > 1:
        names = ", ".join(sorted(os.path.basename(one) for one in holders))
        return HostDependencies(
            problems=[
                f"{len(holders)} packages hold a {CONFIG_DIR}/ directory ({names}); "
                "cannot tell which configuration describes the hosts"
            ]
        )

    config_package = holders[0]
    definitions, shared_problem = _component_definitions(config_package)

    derived = HostDependencies()
    if shared_problem:
        derived.problems.append(shared_problem)

    own_name = os.path.basename(os.path.abspath(config_package))
    for name, hosts in _assignments(config_package, derived).items():
        definition = definitions.get(name)
        if definition is None:
            derived.unresolved.append(name)
            continue
        # scan_from follows the component's `package:` + `launch_file:` edge into the launch
        # tree, so this picks up what the launch file pulls in, not only what the component names.
        found = set(scan_from(definition).packages) - {own_name}
        for host in hosts:
            derived.packages.setdefault(host, set()).update(found)

    return derived


def _assignments(
    config_package: str, derived: HostDependencies
) -> dict[str, list[str]]:
    """`component -> hosts`, unioned over every configuration in the package.

    Unioned rather than read one file at a time: a robot may be started with any of them, so a
    package needed by only one configuration is still a package the host needs installed.
    """
    found: dict[str, list[str]] = {}

    for path in sorted(glob.glob(os.path.join(config_package, CONFIG_DIR, "*.y*ml"))):
        try:
            with open(path, encoding="utf-8") as f:
                document = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError) as e:
            derived.problems.append(f"{os.path.basename(path)} could not be read: {e}")
            continue

        if not isinstance(document, dict):
            continue

        for entry in document.get("components") or []:
            if not isinstance(entry, dict) or not entry.get("name"):
                continue
            hosts = entry.get("hosts", entry.get("host"))
            hosts = [hosts] if isinstance(hosts, str) else hosts
            if not isinstance(hosts, list):
                continue
            on = found.setdefault(str(entry["name"]), [])
            on.extend(str(one) for one in hosts if str(one) not in on)

    return found


def _component_definitions(config_package: str) -> tuple[dict[str, str], str | None]:
    """`component name -> its definition file`, from the repository then the shared library.

    The repository wins, so a robot may override a shared component. Both trees are searched to
    any depth; components are routinely grouped into subdirectories.
    """
    found: dict[str, str] = {}

    share = default_share_lookup(SHARED_COMPONENTS)
    problem = None
    if share is None:
        problem = (
            f"{SHARED_COMPONENTS} is not installed, so components it defines cannot be read - "
            "source the workspace to complete the derivation"
        )
    else:
        _collect(share, found)

    # Second, so the repository's own definitions overwrite the shared ones.
    _collect(os.path.join(config_package, COMPONENT_DIR), found)

    return found, problem


def _collect(root: str, into: dict[str, str]) -> None:
    for path in sorted(glob.glob(os.path.join(root, "**", "*.y*ml"), recursive=True)):
        into[os.path.splitext(os.path.basename(path))[0]] = path


# ── What one run needs to know about the install sets in front of it ─────────────────────


@dataclass
class LaunchManagerContext:
    """The install sets in this run, and what each host turns out to need."""

    hosts: HostDependencies
    config_package: str
    delegated: frozenset[str]
    """Every dependency declared by an install set.

    A launch file in the configuration package may name any of these: they are declared, just
    not by the package the launch file lives in. Together they are what stops the whole
    arrangement reading as one long list of missing dependencies.
    """

    claimed: frozenset[str]
    """The hosts some package claims. A host in a configuration but not here has no install
    set, so nothing declares what it runs."""

    @property
    def unclaimed(self) -> dict[str, set[str]]:
        return {
            host: packages
            for host, packages in self.hosts.packages.items()
            if host not in self.claimed
        }


def context_for(package_xml_files: list[str]) -> LaunchManagerContext | None:
    """Read the install sets in this run, or None if there are none.

    Returns None the moment no manifest claims a host, before parsing anything or walking a
    single launch file - a repository that does not use this pays only for a substring search.
    """
    claiming = [one for one in package_xml_files if _mentions_host_export(one)]
    if not claiming:
        return None

    # A manifest that is unreadable here is unreadable everywhere, and the schema step reports
    # it - so a parse failure is skipped rather than raised a second time.
    delegated: set[str] = set()
    claimed: set[str] = set()
    for path in claiming:
        try:
            parser = ET.XMLParser(no_network=True, resolve_entities=False)
            root = ET.parse(path, parser).getroot()
        except (OSError, ET.XMLSyntaxError):
            continue
        host = declared_host(root)
        if host is None:
            continue
        claimed.add(host)
        delegated.update(retrieve_exec_dependencies(root))

    package_dirs = sorted({os.path.dirname(one) for one in package_xml_files})
    derived = derive(package_dirs)
    config_package = find_config_package(package_dirs) or ""

    return LaunchManagerContext(
        hosts=derived,
        config_package=config_package,
        delegated=frozenset(delegated),
        claimed=frozenset(claimed),
    )


def _mentions_host_export(package_xml: str) -> bool:
    try:
        with open(package_xml, encoding="utf-8", errors="replace") as f:
            return HOST_EXPORT in f.read()
    except OSError:
        return False
