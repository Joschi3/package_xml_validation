import argparse
import os
from lxml import etree as ET
from copy import deepcopy

try:
    from .helpers.logger import get_logger
except ImportError:
    from helpers.logger import get_logger
import subprocess

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


def validate_xml_with_xmllint(xml_file):
    """Validate XML file against the ROS package_format3.xsd schema using xmllint."""
    schema_url = "http://download.ros.org/schema/package_format3.xsd"
    try:
        result = subprocess.run(
            ["xmllint", "--noout", "--schema", schema_url, xml_file],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            print(f"XML validation error in {xml_file}:\n{result.stderr}")
            return False
        return True
    except FileNotFoundError:
        print(
            "Error: xmllint not found. Please ensure it's installed and in your PATH."
        )
        return False


class PackageXmlFormatter:
    def __init__(self, check_only=False, verbose=False, check_with_xmllint=False):
        self.check_only = check_only
        self.check_with_xmllint = check_with_xmllint
        self.logger = get_logger(__name__, level="verbose" if verbose else "normal")

    def find_package_xml_files(self, paths):
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

    def check_dependency_order(self, root, xml_file):
        """Check and optionally correct the order of dependencies in the package.xml file (with comment preservation using lxml)."""

        dependency_order = [elm[0] for elm in ELEMENTS if "depend" in elm[0]]
        dependencies_with_comments = {dep: [] for dep in dependency_order}
        current_order = []

        # Collect dependencies and track their order
        current_comments = []
        for elem in root:
            if elem.tag is ET.Comment:
                current_comments.append(elem)
            if isinstance(elem.tag, str) and elem.tag in dependency_order:
                dependencies_with_comments[elem.tag].append((elem, current_comments))
                current_comments = []
                current_order.append(elem.tag)

        # Determine correct order for type grouping
        correct_order = []
        for dep_type in dependency_order:
            correct_order.extend([dep_type] * len(dependencies_with_comments[dep_type]))

        # Check type order mismatch
        order_mismatch = False
        if current_order != correct_order:
            self.logger.info(f"Dependency order in {xml_file} is incorrect.")
            order_mismatch = True

        if self.check_only and order_mismatch:
            return False
        # Check alphabetical order within each group
        if current_order == correct_order:
            for dep_type, elem_with_commtents in dependencies_with_comments.items():
                names = [e[0].text for e in elem_with_commtents]
                if names != sorted(names):
                    self.logger.debug(
                        f"Dependency order in {xml_file} is incorrect: {dep_type} elements are not sorted."
                    )
                    order_mismatch = True
                    break

        if self.check_only and order_mismatch:
            self.logger.info(f"Dependency order in {xml_file} is incorrect.")
            return False

        if self.check_only:
            self.logger.info(f"Dependency order in {xml_file} is correct.")
            return True

        if not order_mismatch:
            self.logger.info(f"Dependency order in {xml_file} is correct.")
            return True

        # Remove old dependency elements from root
        for dep_type in dependency_order:
            for elem in dependencies_with_comments[dep_type]:
                for comment in elem[1]:
                    root.remove(comment)
                root.remove(elem[0])

        # Find index of <export> or append at end
        export_index = next(
            (i for i, elem in enumerate(root) if elem.tag == "export"), len(root)
        )
        member_of_group_index = next(
            (i for i, elem in enumerate(root) if elem.tag == "member_of_group"),
            len(root),
        )
        group_dep_index = next(
            (i for i, elem in enumerate(root) if elem.tag == "group_depend"), len(root)
        )

        indendantion = root[0].tail.replace("\n", "")
        # Reinsert sorted dependencies before <export>, <member_of_group>, or <group_depend>
        insert_index = min(export_index, member_of_group_index, group_dep_index)
        for dep_type in dependency_order:
            sorted_elems = sorted(
                dependencies_with_comments[dep_type], key=lambda x: x[0].text
            )
            for i, elem_with_comment in enumerate(sorted_elems):
                if i != len(sorted_elems) - 1:
                    elem_with_comment[0].tail = "\n" + indendantion
                else:
                    elem_with_comment[0].tail = "\n\n" + indendantion
                for comment in elem_with_comment[1]:
                    root.insert(insert_index, comment)
                    insert_index += 1
                root.insert(insert_index, elem_with_comment[0])
                insert_index += 1

        self.logger.info(f"Corrected dependency order in {xml_file}.")
        return False

    def check_for_duplicates(self, root, xml_file):
        """
        Check for duplicate elements in the XML file.
        -> meaning tag and text are the same
        """
        seen = set()
        duplicates = []
        for elem in root:
            if elem.tag is ET.Comment:
                continue
            combined = elem.tag + elem.text.strip() if elem.text else elem.tag
            if combined in seen:
                duplicates.append(elem)
            else:
                seen.add(combined)

        if duplicates:
            self.logger.info(
                f"Duplicate elements found in {xml_file}: {', '.join(duplicates)}"
            )
            if self.check_only:
                return False

        if self.check_only:
            self.logger.info(f"No duplicate elements found in {xml_file}.")
            return True

        if not duplicates:
            return True

        # Remove duplicates
        for elem in duplicates:
            root.remove(elem)
        return False

    def check_occurrences(self, root, xml_file):
        """
        Check the min/max occurrences of elements in the XML file.
        If self.check_only is True, only check for errors without correcting.
        Otherwise, it removes the extra elements.
        """
        incorrect_occurrences = False
        for elem, min_occurrences, max_occurrences in ELEMENTS:
            count = len(root.findall(elem))
            if count < min_occurrences:
                self.logger.info(
                    f"Error: Element '{elem}' in {xml_file} has fewer than {min_occurrences} occurrences."
                )
                incorrect_occurrences = True
                if self.check_only:
                    return False
            elif max_occurrences is not None and count > max_occurrences:
                self.logger.info(
                    f"Error: Element '{elem}' in {xml_file} has more than {max_occurrences} occurrences."
                )
                incorrect_occurrences = True
                if self.check_only:
                    return False

        if self.check_only:
            self.logger.info(f"Occurrences of elements in {xml_file} are correct.")
            return True

        if not incorrect_occurrences:
            self.logger.debug(f"Occurrences of elements in {xml_file} are correct.")
            return True

        # Correct the occurrences
        for elem, min_occurrence, max_occurrences in ELEMENTS:
            count = len(root.findall(elem))
            if max_occurrences is not None and count > max_occurrences:
                for i in range(count - max_occurrences):
                    root.remove(root.find(elem))
            if count < min_occurrence:
                self.logger.info("Please add the missing element: ", elem)
        return False

    def check_element_order(self, root, xml_file):
        """
        Check if the elements in the XML file are in the expected order,
        ensuring comments remain in front of their respective elements.
        """
        # Define the expected order of elements
        element_order = [elem[0] for elem in ELEMENTS]

        current_order = [elem for elem in root if elem.tag in element_order]
        misplaced_elements = []
        for i, elem in enumerate(current_order):
            if i > 0 and element_order.index(elem.tag) < element_order.index(
                current_order[i - 1].tag
            ):
                misplaced_elements.append(elem)
        if misplaced_elements:
            self.logger.info(f"Element order in {xml_file} is incorrect.")
            self.logger.info(
                f"Misplaced elements: {', '.join([elem.tag for elem in misplaced_elements])}"
            )
            if self.check_only:
                return False

        if self.check_only:
            self.logger.info(f"Element order in {xml_file} is correct.")
            return True

        if not misplaced_elements:
            return True

        # Helper function to determine the sort key
        def sort_key(elem):
            try:
                return element_order.index(elem.tag)
            except ValueError:
                # Place unexpected elements at the end
                return len(element_order)

        # Extract elements and their preceding comments
        elements_with_comments = []
        current_comments = []

        for elem in root:
            if elem.tag is ET.Comment:
                current_comments.append(deepcopy(elem))
                self.logger.error(f"Found comment: {elem.text}")
            else:
                elements_with_comments.append((deepcopy(elem), current_comments))
                current_comments = []

        # Sort the elements based on the expected order
        elements_with_comments.sort(key=lambda x: sort_key(x[0]))

        # Clear the root and reinsert elements with their comments
        for elem in root:
            root.remove(elem)
        for elem, comments in elements_with_comments:
            for comment in comments:
                root.append(comment)
                self.logger.debug(f"Reinserted comment: {comment.text}")
            root.append(elem)
            self.logger.debug(f"Reinserted element: {elem.tag}")

        return False

    def check_for_empty_lines(self, root, xml_file):
        """
        Check for empty lines in the XML file.
        Make sure there are no more than one empty line between elements.
        """

        found_empty_lines = False
        for elm in root:
            if elm.tail and elm.tail.startswith("\n\n\n"):
                self.logger.info(
                    f"Error: More than one empty line found in {xml_file}."
                )
                found_empty_lines = True
                if self.check_only:
                    return False

        if self.check_only:
            return True
        if not found_empty_lines:
            self.logger.debug(f"No empty lines found in {xml_file}.")
            return True

        # make sure there is no more than one empty line in the xml file
        for elm in root:
            while (
                elm.tail and isinstance(elm.tail, str) and elm.tail.startswith("\n\n\n")
            ):
                elm.tail = elm.tail[1:]
        return False

    def check_and_format_files(self, package_xml_files):
        """Check and format package.xml files if self.check_only is False."""
        all_valid = True
        for xml_file in package_xml_files:
            self.logger.info(f"Processing {xml_file}...")

            parser = ET.XMLParser()
            tree = ET.parse(xml_file, parser)
            root = tree.getroot()

            if not self.check_for_duplicates(root, xml_file):
                all_valid = False
                self.logger.debug(f"Duplicate elements found in {xml_file}.")

            if not self.check_occurrences(root, xml_file):
                all_valid = False
                self.logger.debug(
                    f"Occurrences of elements in {xml_file} are incorrect."
                )

            if not self.check_element_order(root, xml_file):
                all_valid = False
                self.logger.debug(f"Element order in {xml_file} is incorrect.")

            if not self.check_dependency_order(root, xml_file):
                all_valid = False
                self.logger.debug(f"Dependency order in {xml_file} is incorrect.")

            if not all_valid and not self.check_only:
                # Write back to file
                tree.write(
                    xml_file, encoding="utf-8", xml_declaration=True, pretty_print=True
                )
            if self.check_with_xmllint:
                if not validate_xml_with_xmllint(xml_file):
                    self.logger.error(f"XML validation failed {xml_file}.")
                    all_valid = False
                else:
                    self.logger.info(f"XML validation passed {xml_file}.")
        return all_valid

    def check_and_format(self, src):
        package_xml_files = self.find_package_xml_files(src)
        if not package_xml_files:
            self.logger.info("No package.xml files found in the provided paths.")
            return
        return self.check_and_format_files(package_xml_files)


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

    parser.add_argument("--verbose", action="store_true", help="Enable verbose output.")

    parser.add_argument(
        "--check_with_xmllint", action="store_true", help="Check XML with xmllint."
    )

    args = parser.parse_args()

    formatter = PackageXmlFormatter(
        check_only=args.check,
        verbose=args.verbose,
        check_with_xmllint=args.check_with_xmllint,
    )

    if args.file:
        # Process the one file given via --file
        valid = formatter.check_and_format_files([args.file])
    else:
        # Process whatever is found in src
        valid = formatter.check_and_format(args.src)
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
