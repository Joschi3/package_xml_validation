import argparse
import os
import subprocess
from lxml import etree as ET


# Order and min/max occurrences of elements
ELEMENTS = [
    ("name", 1, 1),
    ("version", 1, 1),
    ("description", 1, 1),
    ("maintainer", 1, None),
    ("license", 1, None),
    ("url", 0, None),
    ("author", 0, None),
    ("buildtool_depend", 0, None),
    ("buildtool_export_depend", 0, None),
    ("build_depend", 0, None),
    ("build_export_depend", 0, None),
    ("depend", 0, None),
    ("exec_depend", 0, None),
    ("doc_depend", 0, None),
    ("test_depend", 0, None),
    ("group_depend", 0, None),
    ("member_of_group", 0, None),
    ("export", 0, 1),
]


def find_package_xml_files(paths):
    """Locate all package.xml files within the provided paths."""
    package_xml_files = []
    for path in paths:
        if os.path.isfile(path) and os.path.basename(path) == "package.xml":
            package_xml_files.append(path)
        elif os.path.isdir(path):
            for root, _, files in os.walk(path):
                if "package.xml" in files:
                    package_xml_files.append(os.path.join(root, "package.xml"))
    return package_xml_files


def check_dependency_order(root, xml_file, check_only):
    """Check and optionally correct the order of dependencies in the package.xml file (with comment preservation using lxml)."""

    dependency_order = [elm[0] for elm in ELEMENTS if "depend" in elm[0]]
    dependencies = {dep: [] for dep in dependency_order}
    current_order = []

    # Collect dependencies and track their order
    for elem in root:
        if isinstance(elem.tag, str) and elem.tag in dependency_order:
            dependencies[elem.tag].append(elem)
            current_order.append(elem.tag)

    # Determine correct order for type grouping
    correct_order = []
    for dep_type in dependency_order:
        correct_order.extend([dep_type] * len(dependencies[dep_type]))

    # Check type order mismatch
    if current_order != correct_order and check_only:
        print(f"Dependency order in {xml_file} is incorrect.")
        return False

    # Check alphabetical order within each group
    if current_order == correct_order:
        for dep_type, elems in dependencies.items():
            names = [e.text for e in elems]
            if names != sorted(names):
                if check_only:
                    print(
                        f"Dependency group '{dep_type}' in {xml_file} is not alphabetically sorted."
                    )
                    return False

    if check_only:
        print(f"Dependency order in {xml_file} is correct.")
        return True

    # Remove old dependency elements from root
    for dep_type in dependency_order:
        for elem in dependencies[dep_type]:
            root.remove(elem)

    # Find index of <export> or append at end
    export_index = next(
        (i for i, elem in enumerate(root) if elem.tag == "export"), len(root)
    )
    member_of_group_index = next(
        (i for i, elem in enumerate(root) if elem.tag == "member_of_group"), len(root)
    )
    group_dep_index = next(
        (i for i, elem in enumerate(root) if elem.tag == "group_depend"), len(root)
    )

    indendantion = root[0].tail.replace("\n", "")
    # Reinsert sorted dependencies before <export>, <member_of_group>, or <group_depend>
    insert_index = min(export_index, member_of_group_index, group_dep_index)
    for dep_type in dependency_order:
        sorted_elems = sorted(dependencies[dep_type], key=lambda x: x.text)
        for i, elem in enumerate(sorted_elems):
            if i != len(sorted_elems) - 1:
                elem.tail = "\n" + indendantion
            else:
                elem.tail = "\n\n" + indendantion
            root.insert(insert_index, elem)
            insert_index += 1

    print(f"Corrected dependency order in {xml_file}.")
    return True


def check_for_duplicates(root, xml_file, check_only):
    """
    Check for duplicate elements in the XML file.
    -> meaning tag and text are the same
    """
    seen = set()
    duplicates = []
    for elem in root:
        if elem.tag + elem.text.strip() in seen:
            duplicates.append(elem)
        else:
            seen.add(elem.tag + elem.text.strip())

    if duplicates:
        print(f"Duplicate elements found in {xml_file}: {', '.join(duplicates)}")
        if check_only:
            return False

    if check_only:
        print(f"No duplicate elements found in {xml_file}.")
        return True
    # Remove duplicates
    for elem in duplicates:
        root.remove(elem)
    return False


def check_occurrences(root, xml_file, check_only):
    """
    Check the min/max occurrences of elements in the XML file.
    If check_only is True, only check for errors without correcting.
    Otherwise, it removes the extra elements.
    """
    for elem, min_occurrences, max_occurrences in ELEMENTS:
        count = len(root.findall(elem))
        if count < min_occurrences:
            print(
                f"Error: Element '{elem}' in {xml_file} has fewer than {min_occurrences} occurrences."
            )
            if check_only:
                return False
        elif max_occurrences is not None and count > max_occurrences:
            print(
                f"Error: Element '{elem}' in {xml_file} has more than {max_occurrences} occurrences."
            )
            if check_only:
                return False

    if check_only:
        print(f"Occurrences of elements in {xml_file} are correct.")
        return True

    # Correct the occurrences
    for elem, min_occurrence, max_occurrences in ELEMENTS:
        count = len(root.findall(elem))
        if max_occurrences is not None and count > max_occurrences:
            for i in range(count - max_occurrences):
                root.remove(root.find(elem))
        if count < min_occurrence:
            print("Please add the missing element: ", elem)
    return True


def check_element_order(root, xml_file, check_only):
    """
    Check if the elements in the XML file are in the expected order.
    """

    def find_last_index(root, tag):
        # return the last index of the tag in the root
        for i in range(len(root) - 1, -1, -1):
            if root[i].tag == tag:
                return i
        return -1

    element_order = [elem[0] for elem in ELEMENTS]
    current_order = [elem for elem in root if elem.tag in element_order]
    misplaced_elements = []
    for i, elem in enumerate(current_order):
        if i > 0 and element_order.index(elem.tag) < element_order.index(
            current_order[i - 1].tag
        ):
            misplaced_elements.append(elem)
    if misplaced_elements:
        print(f"Element order in {xml_file} is incorrect.")
        print(
            f"Misplaced elements: {', '.join([elem.tag for elem in misplaced_elements])}"
        )
        if check_only:
            return False

    if check_only:
        print(f"Element order in {xml_file} is correct.")
        return True

    # Correct the order
    for elem in misplaced_elements:
        root.remove(elem)
        if elem.tag == "name":  # special case for first element
            prev_index = -1
        else:
            prev_index = find_last_index(
                root, element_order[element_order.index(elem.tag) - 1]
            )
        root.insert(prev_index + 1, elem)

    return True


def check_for_empty_lines(root, xml_file, check_only):
    """
    Check for empty lines in the XML file.
    Make sure there are no more than one empty line between elements.
    """

    for elm in root:
        if elm.tail and elm.tail.startswith("\n\n\n"):
            print(f"Error: More than one empty line found in {xml_file}.")
            if check_only:
                return False

    if check_only:
        return True

    # make sure there is no more than one empty line in the xml file
    for elm in root:
        while elm.tail and isinstance(elm.tail, str) and elm.tail.startswith("\n\n\n"):
            elm.tail = elm.tail[1:]


def check_and_format_files(package_xml_files, check_only):
    """Check and format package.xml files if check_only is False."""
    all_valid = True
    for xml_file in package_xml_files:
        print(f"Processing {xml_file}...")

        parser = ET.XMLParser()
        tree = ET.parse(xml_file, parser)
        root = tree.getroot()

        if not check_for_duplicates(root, xml_file, check_only):
            all_valid = False

        if not check_occurrences(root, xml_file, check_only):
            all_valid = False

        if not check_element_order(root, xml_file, check_only):
            all_valid = False

        if not check_dependency_order(root, xml_file, check_only):
            all_valid = False

        if not all_valid and not check_only:
            # Write back to file
            tree.write(
                xml_file, encoding="utf-8", xml_declaration=True, pretty_print=True
            )
    return all_valid


def check_and_format(src, check_only):
    package_xml_files = find_package_xml_files(src)
    if not package_xml_files:
        print("No package.xml files found in the provided paths.")
        return
    return check_and_format_files(package_xml_files, check_only)


def main():
    parser = argparse.ArgumentParser(
        description="Validate and check ordering of ROS package.xml files."
    )
    parser.add_argument(
        "src", nargs="*", help="List of files or directories to process."
    )
    parser.add_argument(
        "--check", action="store_true", help="Only check for errors without correcting."
    )
    parser.add_argument(
        "--file",
        help="Path to a single XML file to process. If provided, 'src' arguments are ignored.",
    )

    args = parser.parse_args()

    if args.file:
        # Process the one file given via --file
        valid = check_and_format_files([args.file], args.check)
    else:
        # Process whatever is found in src
        valid = check_and_format(args.src, args.check)
    # if not valid exit with error code
    if not valid and args.check:
        print("Some package.xml files have issues. Please review the messages above.")
        exit(1)
    elif not valid:
        print("Corrected package.xml files.")
        exit(1)
    else:
        print("All package.xml files are valid and correctly ordered.")


if __name__ == "__main__":
    main()
