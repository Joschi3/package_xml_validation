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
from typing import List
import xml.etree.ElementTree as ET
from helpers.cmake_parsers import retrieve_cmake_dependencies
from helpers.rosdep_validator import check_rosdeps


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


def collect_cmake_package_pairs(cmake_files: List[Path], package_xmls: List[Path]):
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
    dep_name = "test " if is_test else ""
    print(f"\t {source} -> {target} missing {dep_name}dependencies:")
    for dep in missing_in_target_deps:
        print(f"\t\t{dep}")


def compare_if_missing(source, target):
    # in priniciple missing_in_target = set(source) - set(target), but robust to
    # lib<name> vs <name> and <name>-dev vs <name>
    missing_in_target = set()
    for dep in source:
        exists = dep in target
        if exists:
            continue
        if dep.startswith("lib"):
            exists = dep[3:] in target
        else:
            exists = f"lib{dep}" in target
        if exists:
            continue
        if dep.endswith("-dev"):
            exists = dep[:-4] in target
        else:
            exists = f"{dep}-dev" in target
        if not exists:
            missing_in_target.add(dep)
    return missing_in_target


def validate(paths, check_rosdep=False, verbose=False):
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
        deps_missing_in_cmake = compare_if_missing(main_deps_xml, main_deps_cmake)
        deps_missing_in_xml = compare_if_missing(main_deps_cmake, main_deps_xml)
        test_deps_missing_in_cmake = compare_if_missing(test_deps_xml, test_deps_cmake)
        test_deps_missing_in_xml = compare_if_missing(test_deps_cmake, test_deps_xml)

        if (
            deps_missing_in_cmake
            or deps_missing_in_xml
            or test_deps_missing_in_cmake
            or test_deps_missing_in_xml
        ):
            print(f"{package_name}")

        if deps_missing_in_cmake:
            print_missing("package.xml", "CMakeLists.txt", deps_missing_in_cmake)
        if deps_missing_in_xml:
            print_missing("CMakeLists.txt", "package.xml", deps_missing_in_xml)
        if test_deps_missing_in_cmake:
            print_missing(
                "package.xml", "CMakeLists.txt", test_deps_missing_in_cmake, True
            )
        if test_deps_missing_in_xml:
            print_missing(
                "CMakeLists.txt", "package.xml", test_deps_missing_in_xml, True
            )
        if (
            deps_missing_in_cmake
            or deps_missing_in_xml
            or test_deps_missing_in_cmake
            or test_deps_missing_in_xml
        ):
            print("\n")

        # make sure the listed keys in the package.xml are resolvable
        if check_rosdep:
            unresolvable_deps = check_rosdeps(main_deps_xml + test_deps_xml)
            if unresolvable_deps:
                print(f"Could not resolve dependencies in {package_name}:")
                for dep in unresolvable_deps:
                    print(f"\t{dep}")
                print("\n")

    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description="ROS2 dependency validator.")
    parser.add_argument(
        "--check_rosdeps",
        action="store_true",
        default=False,
        help="If set tests whether rosdep is able to resolve the listed dependencies.",
    )
    parser.add_argument(
        "SRC",
        nargs="*",
        help="List of files or directories to check. If empty or '.', we scan the current directory.",
    )

    args = parser.parse_args()
    validate(args.SRC, args.check_rosdeps)


if __name__ == "__main__":
    # main()

    paths = ["/home/aljoscha-schmidt/hector/src/"]
    validate(paths, True, False)
