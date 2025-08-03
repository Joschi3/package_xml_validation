#!/usr/bin/env python3
"""
find_launch_dependencies.py

Recursively search a ROS 2 package's launch/ folder and extract
all referenced ROS 2 package names via a small set of regexes.
"""

import os
import re
import argparse


REGEX_EXPR = [
    # YAML-style:  pkg: <pkg_name>
    r"pkg\s*:\s*['\"]?([A-Za-z0-9_]+)['\"]?",
    # Hector launch component:  package: <pkg_name>
    r"package\s*:\s*['\"]?([A-Za-z0-9_]+)['\"]?",
    # Python Node call: Node(..., package='<pkg_name>', ...)
    r"Node\s*\(\s*[^)]*?package\s*=\s*['\"]?([A-Za-z0-9_]+)['\"]?",
    # XML node tag: <node pkg="foo" ...>
    r"<node[^>]*?\bpkg\s*=\s*['\"]?([A-Za-z0-9_]+)['\"]?",
    # get_package_share_directory('foo')
    r"get_package_share_directory\(\s*['\"]([A-Za-z0-9_]+)['\"]\s*\)",
    # FindPackageShare('foo')
    r"FindPackageShare\(\s*['\"]([A-Za-z0-9_]+)['\"]\s*\)",
    # FindPackageShare(package='foo')
    r"FindPackageShare\(\s*package\s*=\s*['\"]([A-Za-z0-9_]+)['\"]\s*\)",
    # $(find-pkg-share foo)
    r"\$\(\s*find-pkg-share\s+([A-Za-z0-9_]+)\s*\)",
]

# Compile once for speed
COMPILED = [re.compile(rx) for rx in REGEX_EXPR]


def scan_file(path, found: set[str], verbose: bool = False):
    """Apply every regex to the file and add matches to `found`."""
    try:
        with open(path, encoding="utf-8") as f:
            text = f.read()
    except (UnicodeDecodeError, OSError):
        return

    for i, rx in enumerate(COMPILED):
        for m in rx.finditer(text):
            found.add(m.group(1))
            if verbose:
                print(
                    f"Found package '{m.group(1)}' in {os.path.basename(path)} with regex {REGEX_EXPR[i]}"
                )


def scan_files(launch_dir: str, verbose: bool = False) -> list[str]:
    if not os.path.isdir(launch_dir):
        print(f"Error: '{launch_dir}' is not a directory.")
        return []

    pkgs = set()

    for root, _, files in os.walk(launch_dir):
        for fn in files:
            if fn.endswith((".py", ".xml", ".yaml", ".launch", ".yml")):
                scan_file(os.path.join(root, fn), pkgs, verbose)
    return list(pkgs)


def parse_args():
    p = argparse.ArgumentParser(
        description="Extract referenced ROS 2 package names from launch files via regex."
    )
    p.add_argument("launch_dir", help="Path to your package's launch/ directory")
    p.add_argument(
        "-v", "--verbose", action="store_true", help="Print found packages to stdout"
    )
    # parse args
    args = p.parse_args()
    pkgs = scan_files(args.launch_dir)
    if args.verbose:
        print("Found packages:")
        for pkg in sorted(pkgs):
            print(f"  - {pkg}")


if __name__ == "__main__":
    # parse_args()
    file = "/home/aljoscha-schmidt/hector/src/simulation_scenario_robocup_gazebo/launch/launch_world.launch.py"
    found = set()
    scan_file(file, found, verbose=True)
    print("Found packages:")
    for pkg in sorted(found):
        print(f"  - {pkg}")
