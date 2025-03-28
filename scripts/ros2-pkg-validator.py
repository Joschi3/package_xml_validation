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
from helpers.cmake_parsers import retrieve_cmake_dependencies


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
    # make sure all files e
    existing_pairs = []
    for package_xml, cmake_file in pairs:
        if package_xml.exists() and cmake_file.exists():
            existing_pairs.append((package_xml, cmake_file))
    return existing_pairs

def print_missing(source, target, missing_in_target_deps, is_test=False):
    print(f"\t {source} -> {target} missing {"test" if is_test else ''}dependencies:")
    for dep in missing_in_target_deps:
        print(f"\t\t{dep}")


def validate(paths, check=False, verbose=False):
    # If SRC is empty or is just ['.'], treat it as "scan current directory".
    if not paths or paths == ["."]:
        src_paths = [Path(".").resolve()]
    else:
        src_paths = [Path(s).resolve() for s in paths]

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
        package_name = package_xml.parent.name
        main_deps_xml, test_deps_xml = parse_package_xml(package_xml)
        main_deps_cmake, test_deps_cmake = retrieve_cmake_dependencies(cmake_list)
        if verbose:
            print(f"Package.xml: {package_xml}")
            print(f"CMakeLists.txt: {cmake_list}")
            print(f"Main deps XML: {main_deps_xml}")
            print(f"Test deps XML: {test_deps_xml}")
            print(f"Main deps CMake: {main_deps_cmake}")
            print(f"Test deps CMake: {test_deps_cmake}")
        # Check for discrepancies between package.xml and CMakeLists.txt
        deps_missing_in_cmake = set(main_deps_xml) - set(main_deps_cmake )
        deps_missing_in_xml = set(main_deps_cmake) - set(main_deps_xml)
        test_deps_missing_in_cmake = set(test_deps_xml) - set(test_deps_cmake)
        test_deps_missing_in_xml = set(test_deps_cmake) - set(test_deps_xml)

        if deps_missing_in_cmake or deps_missing_in_xml or test_deps_missing_in_cmake or test_deps_missing_in_xml:
            print(f"{package_name}")

        if deps_missing_in_cmake:
            print_missing("package.xml", "CMakeLists.txt", deps_missing_in_cmake)
        if deps_missing_in_xml:
            print_missing("CMakeLists.txt", "package.xml", deps_missing_in_xml)
        if test_deps_missing_in_cmake:
            print_missing("package.xml", "CMakeLists.txt", test_deps_missing_in_cmake, True)
        if test_deps_missing_in_xml:
            print_missing("CMakeLists.txt", "package.xml", test_deps_missing_in_xml, True)
        if deps_missing_in_cmake or deps_missing_in_xml or test_deps_missing_in_cmake or test_deps_missing_in_xml:
            print("\n")
    # Placeholder: If --check was given, you'd do some error reporting logic here
    # (e.g. comparing the sets of deps in the CMake vs. package.xml and printing mismatches)
    #
    # But for now, we just exit 0 for success
    sys.exit(0)

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
    validate(args.SRC, args.check)
 


if __name__ == "__main__":
    #main()

    paths = ["/home/aljoscha-schmidt/hector/src/"]
    validate(paths, False, False)