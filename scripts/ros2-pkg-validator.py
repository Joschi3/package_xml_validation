#!/usr/bin/env python3
"""
ros2-dependency-validator.py

A simple script that:
1. Finds package.xml and/or CMakeLists.txt files in user-specified locations.
2. Parses dependencies (main + test) from each.
3. Optionally (via --check) only reports findings without any attempt to fix them.
"""

import argparse
import sys
import os
from pathlib import Path
import re
from typing import List
import xml.etree.ElementTree as ET


def parse_package_xml(xml_path: Path):
    """
    Parse <package.xml> for normal dependencies and test dependencies.

    Returns:
        (main_deps, test_deps)
        main_deps: list of strings for normal (build, etc.) dependencies
        test_deps: list of strings for test_depend
    """
    main_deps = []
    test_deps = []

    if not xml_path.exists():
        return main_deps, test_deps

    tree = ET.parse(xml_path)
    root = tree.getroot()

    main_tags = {"depend", "build_depend", "build_export_depend", "buildtool_depend"}
    test_tags = {"test_depend"}

    for child in root:
        tag_name = child.tag.lower()
        text_value = (child.text or "").strip()
        if tag_name in main_tags:
            main_deps.append(text_value)
        elif tag_name in test_tags:
            test_deps.append(text_value)

    return main_deps, test_deps


def parse_cmake_list(cmake_path: Path):
    """
    Parse a CMakeLists.txt for dependencies discovered via find_package().
    We distinguish between "normal" dependencies and "test" dependencies
    if they appear within an `if(BUILD_TESTING)` block.
    
    Returns:
        (main_deps, test_deps)
    """
    main_deps = []
    test_deps = []

    if not cmake_path.exists():
        return main_deps, test_deps

    # We'll track whether we're inside an `if(BUILD_TESTING)` block
    # This is a naive approach. Real CMake can have nested if() statements, etc.
    in_test_block = False
    if_stack = []

    # Regex to match lines like: find_package(PackageName REQUIRED ...)
    find_package_pattern = re.compile(r'^\s*find_package\s*\(\s*([A-Za-z0-9_\-]+)')

    with cmake_path.open("r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()

            # Check for if/endif transitions
            # We'll do a simplistic check: if(...) lines and endif(...)
            if stripped.lower().startswith("if(") and "build_testing" in stripped.lower():
                # Entering a test block
                if_stack.append("BUILD_TESTING")
                in_test_block = True

            elif stripped.lower().startswith("if("):
                # Some other if() we don't specifically track
                if_stack.append("OTHER")

            elif stripped.lower().startswith("endif"):
                if if_stack:
                    popped = if_stack.pop()
                # Recompute whether we are still in a test block
                in_test_block = any(item == "BUILD_TESTING" for item in if_stack)

            # Check for find_package
            match = find_package_pattern.match(stripped)
            if match:
                pkg_name = match.group(1)
                if in_test_block:
                    test_deps.append(pkg_name)
                else:
                    main_deps.append(pkg_name)

    return main_deps, test_deps


def find_files_in_dir(base_dir: Path):
    """
    Recursively find all package.xml or CMakeLists.txt files under base_dir.
    Returns two sets: (xml_files, cmake_files).
    """
    xml_files = set()
    cmake_files = set()
    for root, dirs, files in os.walk(base_dir):
        for file in files:
            if file == "package.xml":
                xml_files.add(Path(root) / file)
            elif file == "CMakeLists.txt":
                cmake_files.add(Path(root) / file)
    return xml_files, cmake_files

def collect_cmake_package_pairs(cmake_files:List[Path], package_xmls:List[Path]):
    # corresponding package.xml and cmake files are in the same directory
    pairs = []
    visited_cmake_files = set()
    for package_xml in package_xmls:
        pairs.append((package_xml, package_xml.parent / "CMakeLists.txt"))
        visited_cmake_files.add(package_xml.parent / "CMakeLists.txt")
    for cmake_file in cmake_files:
        if cmake_file not in visited_cmake_files:
            pairs.append((cmake_file.parent / "package.xml", cmake_file))
    return pairs


def main():
    parser = argparse.ArgumentParser(description="ROS2 dependency validator.")
    parser.add_argument(
        "--check",
        action="store_true",
        default=False,
        help="If set, only report errors (placeholder). No corrections are performed."
    )
    parser.add_argument(
        "SRC",
        nargs="*",
        help="List of files or directories to check. If empty or '.', we scan the current directory."
    )

    args = parser.parse_args()

    # If SRC is empty or is just ['.'], treat it as "scan current directory".
    if not args.SRC or args.SRC == ["."]:
        src_paths = [Path(".").resolve()]
    else:
        src_paths = [Path(s).resolve() for s in args.SRC]

    # Collect unique package.xml and CMakeLists.txt from all provided paths
    all_package_xmls = set()
    all_cmake_lists = set()

    for path in src_paths:
        if path.is_dir():
            # Recursively scan
            xmls, cmakes = find_files_in_dir(path)
            all_package_xmls.update(xmls)
            all_cmake_lists.update(cmakes)
        elif path.is_file():
            if path.name == "package.xml":
                all_package_xmls.add(path)
            elif path.name == "CMakeLists.txt":
                all_cmake_lists.add(path)
        # else: non-existent path is ignored in this example
    

    # If no relevant files found, exit 0 (no error)
    if not all_package_xmls and not all_cmake_lists:
        print("No package.xml or CMakeLists.txt found. Exiting with code 0.")
        sys.exit(0)
    
    # collect pairs of package.xml and CMakeLists.txt files
    pairs = collect_cmake_package_pairs(list(all_cmake_lists), list(all_package_xmls))

    for package_xml, cmake_list in pairs:
        main_deps_xml, test_deps_xml = parse_package_xml(package_xml)
        main_deps_cmake, test_deps_cmake = parse_cmake_list(cmake_list)

        # Check for discrepancies between package.xml and CMakeLists.txt
        deps_missing_in_cmake = set(main_deps_xml) - set(main_deps_cmake )
        deps_missing_in_xml = set(main_deps_cmake) - set(main_deps_xml)
        test_deps_missing_in_cmake = set(test_deps_xml) - set(test_deps_cmake)
        test_deps_missing_in_xml = set(test_deps_cmake) - set(test_deps_xml)

        if deps_missing_in_cmake:
            print(f"Dependencies missing in CMakeLists.txt: {deps_missing_in_cmake}")
        if deps_missing_in_xml:
            print(f"Dependencies missing in package.xml: {deps_missing_in_xml}")
        if test_deps_missing_in_cmake:
            print(f"Test dependencies missing in CMakeLists.txt: {test_deps_missing_in_cmake}")
        if test_deps_missing_in_xml:
            print(f"Test dependencies missing in package.xml: {test_deps_missing_in_xml}")
    
    # Placeholder: If --check was given, you'd do some error reporting logic here
    # (e.g. comparing the sets of deps in the CMake vs. package.xml and printing mismatches)
    #
    # But for now, we just exit 0 for success
    sys.exit(0)


if __name__ == "__main__":
    main()
