import argparse
import os
import subprocess
from lxml import etree as ET

# Define the expected order of dependency tags
DEPENDENCY_ORDER = [
    'buildtool_depend',
    'buildtool_export_depend',
    'build_depend',
    'build_export_depend',
    'depend',
    'exec_depend',
    'doc_depend',
    'test_depend'
]

def find_package_xml_files(paths):
    """Locate all package.xml files within the provided paths."""
    package_xml_files = []
    for path in paths:
        if os.path.isfile(path) and os.path.basename(path) == 'package.xml':
            package_xml_files.append(path)
        elif os.path.isdir(path):
            for root, _, files in os.walk(path):
                if 'package.xml' in files:
                    package_xml_files.append(os.path.join(root, 'package.xml'))
    return package_xml_files

def validate_xml_with_xmllint(xml_file):
    """Validate XML file against the ROS package_format3.xsd schema using xmllint."""
    schema_url = "http://download.ros.org/schema/package_format3.xsd"
    try:
        result = subprocess.run(
            ['xmllint', '--noout', '--schema', schema_url, xml_file],
            capture_output=True,
            text=True
        )
        if result.returncode != 0:
            print(f"XML validation error in {xml_file}:\n{result.stderr}")
            return False
        return True
    except FileNotFoundError:
        print("Error: xmllint not found. Please ensure it's installed and in your PATH.")
        return False

def check_dependency_order(xml_file, check_only):
    """Check and optionally correct the order of dependencies in the package.xml file (with comment preservation using lxml)."""
    parser = ET.XMLParser()
    tree = ET.parse(xml_file, parser)
    root = tree.getroot()

    dependencies = {dep: [] for dep in DEPENDENCY_ORDER}
    current_order = []

    # Collect dependencies and track their order
    for elem in root:
        if isinstance(elem.tag, str) and elem.tag in DEPENDENCY_ORDER:
            dependencies[elem.tag].append(elem)
            current_order.append(elem.tag)

    # Determine correct order for type grouping
    correct_order = []
    for dep_type in DEPENDENCY_ORDER:
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
                    print(f"Dependency group '{dep_type}' in {xml_file} is not alphabetically sorted.")
                    return False

    if check_only:
        return True

    # Remove old dependency elements from root
    for dep_type in DEPENDENCY_ORDER:
        for elem in dependencies[dep_type]:
            root.remove(elem)

    # Find index of <export> or append at end
    export_index = next((i for i, elem in enumerate(root) if elem.tag == 'export'), len(root))

    # Reinsert sorted dependencies before <export>
    insert_index = export_index
    for dep_type in DEPENDENCY_ORDER:
        sorted_elems = sorted(dependencies[dep_type], key=lambda x: x.text)
        for elem in sorted_elems:
            root.insert(insert_index, elem)
            insert_index += 1

    # Write back to file
    tree.write(xml_file, encoding='utf-8', xml_declaration=True, pretty_print=True)
    print(f"Corrected dependency order in {xml_file}.")
    return True

def check_and_format(src, check_only):
    package_xml_files = find_package_xml_files(src)
    if not package_xml_files:
        print("No package.xml files found in the provided paths.")
        return

    all_valid = True
    for xml_file in package_xml_files:
        print(f"Processing {xml_file}...")

        # Validate XML structure
        if not validate_xml_with_xmllint(xml_file):
            all_valid = False
            continue

        # Check and possibly correct dependency order
        if not check_dependency_order(xml_file, check_only):
            all_valid = False

    if all_valid:
        print("All package.xml files are valid and correctly ordered.")
    else:
        print("Some package.xml files have issues. Please review the messages above.")

def main():
    parser = argparse.ArgumentParser(description="Validate and check ordering of ROS package.xml files.")
    parser.add_argument('src', nargs='+', help="List of files or directories to process.")
    parser.add_argument('--check', action='store_true', help="Only check for errors without correcting.")
    args = parser.parse_args()
    check_and_format(args.src, args.check)

   

if __name__ == "__main__":
    # main()
    paths = ["/home/aljoscha-schmidt/hector/src/hector_base_velocity_manager/hector_base_velocity_manager/package.xml"]
    check_and_format(paths, False)
