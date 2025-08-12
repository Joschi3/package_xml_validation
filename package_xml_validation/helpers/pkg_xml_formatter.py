import lxml.etree as ET
from copy import deepcopy


try:
    from .logger import get_logger
except ImportError:
    from helpers.logger import get_logger

# tuple of (element_name, min_occurrences, max_occurrences)
# min_occurrences is 1 if the element is required, 0 if it can be missing
# max_occurrences is None if there is no limit, otherwise it is a positive integer
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

NEW_LINE_BEFORE = [
    "buildtool_depend",
    "build_depend",
    "depend",
    "exec_depend",
    "doc_depend",
    "test_depend",
    "group_depend",
    "member_of_group",
    "export",
]

NEW_LINE = "\n"


class PackageXmlFormatter:
    def __init__(
        self,
        check_only=False,
        verbose=False,
        check_with_xmllint=False,
        logger=None,
    ):
        self.check_only = check_only
        self.check_with_xmllint = check_with_xmllint
        self.logger = (
            logger
            if logger
            else get_logger(__name__, level="verbose" if verbose else "normal")
        )
        self.encountered_unresolvable_error = False

    def prettyprint(self, element, **kwargs):
        xml = ET.tostring(element, pretty_print=True, **kwargs)
        print(xml.decode(), end="")

    def check_dependency_order(self, root, xml_file):
        """Check and optionally correct the order of dependencies in the package.xml file (with comment preservation using lxml)."""

        dependency_order = [elm[0] for elm in ELEMENTS]
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
            self.logger.error(f"Dependency order in {xml_file} is incorrect.")
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
            self.logger.error(f"Dependency order in {xml_file} is incorrect.")
            return False

        if self.check_only:
            return True

        if not order_mismatch:
            return True

        indentation = root[0].tail.replace(NEW_LINE, "")
        # Remove old dependency elements from root
        for dep_type in dependency_order:
            for elem in dependencies_with_comments[dep_type]:
                for comment in elem[1]:
                    root.remove(comment)
                root.remove(elem[0])
        # filter out empty lists from dependency order
        dependency_order = [
            dep for dep in dependency_order if dependencies_with_comments[dep]
        ]
        insert_index = 0
        for index, dep_type in enumerate(dependency_order):
            sorted_elems = sorted(
                dependencies_with_comments[dep_type], key=lambda x: x[0].text
            )
            for i, elem_with_comment in enumerate(sorted_elems):
                if (
                    i == len(sorted_elems) - 1
                    and index + 1 < len(dependency_order)
                    and dependency_order[index + 1] in NEW_LINE_BEFORE
                ):
                    elem_with_comment[0].tail = "\n\n" + indentation
                else:
                    elem_with_comment[0].tail = NEW_LINE + indentation
                for comment in elem_with_comment[1]:
                    root.insert(insert_index, comment)
                    insert_index += 1
                root.insert(insert_index, elem_with_comment[0])
                insert_index += 1
        root[-1].tail = NEW_LINE

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
                f"Duplicate elements found in {xml_file}: {', '.join([elem.tag for elem in duplicates])}"
            )
            if self.check_only:
                return False

        if self.check_only:
            return True

        if not duplicates:
            return True

        # Remove duplicates
        for elem in duplicates:
            root.remove(elem)
        return False

    def check_element_occurrences(self, root, xml_file):
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
            return True

        if not incorrect_occurrences:
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
            self.logger.error(f"Element order in {xml_file} is incorrect.")
            self.logger.error(
                f"Misplaced elements: {', '.join([elem.tag for elem in misplaced_elements])}"
            )
            if self.check_only:
                return False

        if self.check_only:
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

        last_tail = ""
        indentation = root[0].tail.replace(NEW_LINE, "")
        for elem in root:
            if elem.tag is ET.Comment:
                self.logger.error(f"Found comment: {elem.text}")
                if last_tail and last_tail[-1] == NEW_LINE:
                    # inline comment -> append to previous element
                    elements_with_comments[-1][0].tail = elem.tail
                    elem.tail = NEW_LINE + indentation
                    elements_with_comments[-1][1].append(deepcopy(elem))
                else:
                    # Ensure only one NEW_LINE in elem.tail
                    elem.tail = NEW_LINE + (
                        elem.tail.replace(NEW_LINE, "") if elem.tail else ""
                    )
                    current_comments.append(deepcopy(elem))
            else:
                elements_with_comments.append((deepcopy(elem), current_comments))
                current_comments = []
            last_tail = elem.tail if elem.tail else ""

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

        def remove_inner_newlines(s):
            first_newline_pos = s.find(NEW_LINE)
            last_newline_pos = s.rfind(NEW_LINE)

            if first_newline_pos == -1 or first_newline_pos == last_newline_pos:
                return s

            start = s[: first_newline_pos + 1]
            middle = s[first_newline_pos + 1 : last_newline_pos].replace(NEW_LINE, "")
            end = s[last_newline_pos:]
            return start + middle + end

        found_empty_lines = False
        for elm in root:
            if elm.tail and elm.tail.count(NEW_LINE) > 2:
                self.logger.info(
                    f"Error: More than one empty line found in {xml_file}."
                )
                found_empty_lines = True
                if self.check_only:
                    return False
            if elm.tail is None or elm.tail.count(NEW_LINE) == 0:
                found_empty_lines = True
                self.logger.info(f"Error: Two Elements in the sane line in {xml_file}.")
                if self.check_only:
                    return False

        if self.check_only:
            return True
        if not found_empty_lines:
            return True
        # elements after last \n
        indendantion = root[0].tail[root[0].tail.rfind(NEW_LINE) + 1 :]
        # correct the empty lines & missing newlines
        for elm in root:
            if elm.tail and elm.tail.count(NEW_LINE) > 2:
                elm.tail = remove_inner_newlines(elm.tail)
            elif elm.tail and elm.tail.count(NEW_LINE) == 0:
                elm.tail += NEW_LINE
            elif elm.tail is None:
                elm.tail = NEW_LINE + indendantion
        return False

    def check_indentation(self, root, level=1, indentation="  "):
        """
        Check if the indentation of the XML file is correct.
        recursively checks the indentation of each element.
        """
        is_correct = True

        def check_indentation_string(string, expected_indent) -> bool:
            """The string should be indented with the expected_indent and contain a newline."""
            if not string or not isinstance(string, str):
                return False
            parsed_indentation = string.replace(NEW_LINE, "")
            return parsed_indentation == expected_indent and NEW_LINE in string

        def fix_indentation(string, expected_indent) -> str:
            """Fix the indentation of the string to match the expected_indent."""
            indent = (
                string.replace(" ", "") if string and NEW_LINE in string else NEW_LINE
            )
            return indent + expected_indent

        def check_and_correct(string, expected_indent, name) -> tuple[str, bool]:
            """Check and correct the indentation of the string."""
            if not check_indentation_string(string, expected_indent):
                # self.logger.error(
                #    f"Incorrect indentation for element '{name}'. Expected: '{expected_indent}', Found: '{string.replace(NEW_LINE, '') if string else 'None'}'"
                # )
                if not self.check_only:
                    string = fix_indentation(string, expected_indent)
                return string, True
            return string, False

        root.text, corrected = check_and_correct(
            root.text, indentation * level, f"{root.tag}-text"
        )
        is_correct &= not corrected

        for index, elem in enumerate(root):
            is_last = index == len(root) - 1
            expected_indent = (
                indentation * (level - 1) if is_last else indentation * level
            )
            elem.tail, corrected = check_and_correct(
                elem.tail,
                expected_indent,
                f"{elem.tag}-{elem.text[:15] if elem.text else 'None'}",
            )
            is_correct &= not corrected
            if len(elem) > 0:  # has children
                # check children recursively
                if not self.check_indentation(elem, level + 1, indentation):
                    is_correct = False
            else:
                # make sure there are no new lines in texts
                if elem.text and NEW_LINE in elem.text:
                    self.logger.error(
                        f"Element '{elem.tag}' has new lines in its text: '{elem.text}'"
                    )
                    is_correct = False
                    if not self.check_only:
                        elem.text = elem.text.replace(NEW_LINE, " ").strip()
        if not is_correct and self.check_only:
            self.logger.error(
                "Incorrect indentation found in package.xml. Please fix the indentations."
            )
        elif not is_correct:
            self.logger.warning("Auto-corrected indentation in package.xml.")
        return is_correct

    def check_for_non_existing_tags(self, root, xml_file):
        """Check for non-existing tags in the XML file."""
        non_existing_tags = []
        valid_tags = [e[0] for e in ELEMENTS]
        for elem in root:
            if isinstance(elem.tag, str) and elem.tag not in valid_tags:
                non_existing_tags.append(elem.tag)
        if non_existing_tags:
            self.logger.error(
                f"Non-existing tags found in {xml_file}: {', '.join(non_existing_tags)}"
            )
            return False
        return True

    def retrieve_all_dependencies(self, root):
        """Retrieve all dependencies from the XML file."""
        dependencies = []
        for elem in root:
            if isinstance(elem.tag, str) and "depend" in elem.tag and elem.text:
                dependencies.append(elem.text.strip())
        return dependencies

    def retrieve_build_dependencies(self, root):
        """Retrieve all build dependencies from the XML file."""
        build_deps = [
            "buildtool_depend",
            "buildtool_export_depend",
            "build_depend",
            "build_export_depend",
            "depend",
        ]
        build_dependencies = []
        for elem in root:
            if isinstance(elem.tag, str) and elem.tag in build_deps and elem.text:
                build_dependencies.append(elem.text.strip())
        return build_dependencies

    def retrieve_test_dependencies(self, root):
        """Retrieve all test dependencies from the XML file."""
        test_dependencies = []
        test_deps = ["test_depend", "depend"]
        for elem in root:
            if isinstance(elem.tag, str) and elem.tag in test_deps and elem.text:
                test_dependencies.append(elem.text.strip())
        return test_dependencies

    def retrieve_exec_dependencies(self, root):
        """Retrieve all exec dependencies from the XML file."""
        exec_dependencies = []
        exec_deps = ["exec_depend", "depend"]
        for elem in root:
            if isinstance(elem.tag, str) and elem.tag in exec_deps and elem.text:
                exec_dependencies.append(elem.text.strip())
        return exec_dependencies

    def get_package_name(self, root) -> str | None:
        """Retrieve the package name from the XML file."""
        name_elem = root.find("name")
        if name_elem is not None and name_elem.text:
            return name_elem.text.strip()
        return None

    def add_dependencies(self, root, dependencies, dep_type):
        """Add dependencies to the XML file."""
        dep_types = [dep[0] for dep in ELEMENTS if "depend" in dep[0]]
        elements = [dep[0] for dep in ELEMENTS]
        if dep_type not in dep_types:
            raise ValueError(f"Invalid dependency type: {dep_type}")
        indendantion = root[0].tail.replace(NEW_LINE, "")
        insert_position, first_of_group = 0, 0
        for dep in dependencies:
            new_elem = ET.Element(dep_type)
            new_elem.text = dep
            new_elem.tail = NEW_LINE + indendantion
            # add element to root at correct position -> correct dep group and alphabetical order
            # case 1: dependency group is empty
            if not root.findall(dep_type):
                previous_element = elements[elements.index(dep_type) - 1]
                while not root.findall(previous_element):
                    previous_element = elements[elements.index(previous_element) - 1]
                # find last element with previous_element tag
                last_element_count = 0
                for count, elm in enumerate(root):
                    if isinstance(elm.tag, str) and elm.tag == previous_element:
                        last_element_count = count
                insert_position = last_element_count + 1
                first_of_group = insert_position
            # case 2: dependency group is not empty
            else:
                # assume list is sorted -> insert at correct position
                insert_position = 0
                first_of_group = None
                for i, elm in enumerate(root):
                    if isinstance(elm.tag, str) and elm.tag == dep_type:
                        if first_of_group is None:
                            first_of_group = i
                            insert_position = i
                        if elm.text < new_elem.text:
                            insert_position = i + 1
            root.insert(insert_position, new_elem)
            # adapt empty lines -> in case element prior ends with empty line move it to the new element
            if (
                insert_position > 0
                and first_of_group is not None
                and insert_position > first_of_group
            ):
                previous_element = root[insert_position - 1]
                if previous_element.tail and previous_element.tail.count(NEW_LINE) > 1:
                    new_elem.tail = previous_element.tail
                    previous_element.tail = NEW_LINE + indendantion
            if insert_position < len(root) - 1:
                # if next tag is different than the new element, add empty line
                next_element = root[insert_position + 1]
                if next_element.tag != new_elem.tag:
                    new_elem.tail = "\n\n" + indendantion

    def add_build_type_export(self, root, build_type: str):
        """
        Add the build type export to the XML file.
        If the build type export already exists, it will be updated.
        If it does not exist, it will be created.
        Other exports will not be changed(besides the build_type export).
        """
        indendantion = root[0].tail.replace(NEW_LINE, "")
        export = root.find("export")
        if export is None:
            export = ET.Element("export")
            # get last element in root
            if len(root) > 0:
                last_element = root[-1]
                if last_element.tail:
                    last_element.tail = "\n\n" + indendantion
            export.tail = NEW_LINE
            root.append(export)
        build_type_elem = export.find("build_type")
        if build_type_elem is None:
            build_type_elem = ET.Element("build_type")
            build_type_elem.tail = NEW_LINE + indendantion
            export.append(build_type_elem)
        build_type_elem.text = build_type
        export.text = NEW_LINE + 2 * indendantion

    def add_buildtool_depends(self, root, buildtool: list[str]):
        """
        Add the buildtool_depend to the XML file.
        If the buildtool_depend already exists, it will be updated.
        If it does not exist, it will be created.
        """
        indendantion = root[0].tail.replace(NEW_LINE, "")
        # 1. clear existing buildtool_depend elements
        for elem in root.findall("buildtool_depend"):
            root.remove(elem)
        # 2. insertposition -> after license, url, or author element
        insert_position = 0
        for i, elem in enumerate(root):
            if isinstance(elem.tag, str) and elem.tag == "license":
                insert_position = i + 1
            elif isinstance(elem.tag, str) and elem.tag == "url":
                insert_position = i + 1
            elif isinstance(elem.tag, str) and elem.tag == "author":
                insert_position = i + 1
        # 3. add buildtool_depend elements
        for i, tool in enumerate(buildtool):
            is_last = i == len(buildtool) - 1
            new_elem = ET.Element("buildtool_depend")
            new_elem.text = tool
            new_elem.tail = "\n\n" if is_last else NEW_LINE
            new_elem.tail += indendantion
            root.insert(insert_position, new_elem)
            insert_position += 1

    def add_member_of_group(self, root, group_name: str):
        """Add member_of_group element to the XML file."""
        indendantion = root[0].tail.replace(NEW_LINE, "")
        member_of_group = ET.Element("member_of_group")
        member_of_group.text = group_name
        member_of_group.tail = "\n\n" + indendantion
        # insert position -> right before export or at the end
        insert_position = len(root)
        for i, elem in enumerate(root):
            if isinstance(elem.tag, str) and elem.tag == "export":
                insert_position = i
                break
        root.insert(insert_position, member_of_group)


if __name__ == "__main__":
    # Example usage
    pkg = "/home/aljoscha-schmidt/hector/src/hector_gamepad_manager/hector_gamepad_manager/package.xml"
    formatter = PackageXmlFormatter(
        check_only=False,
        verbose=True,
        check_with_xmllint=True,
    )
