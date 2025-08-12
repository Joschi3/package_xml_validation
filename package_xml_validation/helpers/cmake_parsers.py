from pathlib import Path
import re
from typing import List


def remove_comments(lines: List[str]) -> list[str]:
    """Removes comments from a list of lines."""
    return [line.split("#", 1)[0].strip() for line in lines]


def read_cmake_lines_with_parens_joined(raw_lines: List[str]) -> list[str]:
    """
    Reads a CMake file and joins lines that have an opening '(' without a matching ')'
    until the closing ')' is found. Returns a list of logically complete lines.
    """
    lines = []
    buffer = ""

    def has_balanced_parens(s: str) -> bool:
        """Returns True if the number of ( matches the number of ) in the string."""
        return s.count("(") == s.count(")")

    for raw_line in raw_lines:
        line = raw_line.strip()
        if not line:
            continue  # skip blank lines

        if not buffer:
            buffer = line
        else:
            buffer += " " + line  # continuation from previous line

        if has_balanced_parens(buffer):
            lines.append(buffer)
            buffer = ""

    # If file ends with an unbalanced block, add it anyway
    if buffer:
        lines.append(buffer)

    return lines


def resolve_for_each(raw_lines: List[str]) -> List[str]:
    """Expands CMake's foreach() loops in a list of lines."""
    foreach_stack = []

    # Regex patterns
    set_pattern = re.compile(
        r"^\s*set\s*\(\s*([A-Za-z0-9_]+)\s+(.*?)\)\s*$", re.IGNORECASE
    )
    foreach_pattern = re.compile(
        r"^\s*foreach\s*\(\s*([A-Za-z_][A-Za-z0-9_]*)\s+IN\s+(LISTS|ITEMS)\s+(\$\{\s*[A-Za-z0-9_]+\s*\}|[A-Za-z0-9_]+)\s*\)\s*$",
        re.IGNORECASE,
    )
    endforeach_pattern = re.compile(r"^\s*endforeach\s*\(?.*\)?\s*$", re.IGNORECASE)
    variables = {}
    lines = []
    for line in raw_lines:
        stripped = line.strip()

        # ---------- Parse set(...) lines ----------
        set_match = set_pattern.match(stripped)
        if set_match:
            var_name = set_match.group(1)
            values_str = set_match.group(2)  # everything after var name
            # Expand any references in the remainder
            expanded_vals = values_str.split()
            variables[var_name] = expanded_vals
            lines.append(line)
            continue

        # ---------- Parse foreach(...) lines ----------
        foreach_match = foreach_pattern.match(stripped)
        if foreach_match:
            loop_var = foreach_match.group(1)
            list_var = foreach_match.group(3)
            # If the list_var is known, store the expansion, otherwise empty
            list_var = list_var.strip("${}")
            expanded_vals = variables.get(list_var, [])
            foreach_stack.append((loop_var, expanded_vals))
            continue

        # ---------- Parse endforeach lines ----------
        if endforeach_pattern.match(stripped):
            if foreach_stack:
                foreach_stack.pop()
            continue

        if foreach_stack:
            loop_var, expanded_vals = foreach_stack[-1]
            if loop_var in stripped:
                for val in expanded_vals:
                    lines.append(stripped.replace(f"${{{loop_var}}}", val))
        else:
            lines.append(line)
    return lines


def retrieve_cmake_dependencies(lines: List[str]) -> tuple[List[str], List[str]]:
    if isinstance(lines, Path):
        lines = read_cmake_file(lines)
    main_deps: list[str] = []
    test_deps: list[str] = []

    # We'll track blocks of 'if(BUILD_TESTING)' with a small stack
    if_stack = []
    in_test_block = False

    # Regex patterns
    if_pattern = re.compile(r"^\s*if\s*\((.*)\)\s*$", re.IGNORECASE)
    endif_pattern = re.compile(r"^\s*endif\s*\(?.*\)?\s*$", re.IGNORECASE)

    # We look for find_package(...) statements, which might contain expansions
    # e.g. find_package(${PKG} REQUIRED)
    find_package_pattern = re.compile(
        r"^\s*find_package\s*\(\s*([^)]+)\)\s*$", re.IGNORECASE
    )

    def add_deps(dep_list: list[str], is_test: bool):
        """
        Append dependency names to main_deps or test_deps, depending on is_test.
        """
        if is_test:
            test_deps.extend(dep_list)
        else:
            main_deps.extend(dep_list)

    for line in lines:
        stripped = line.strip()

        # ---------- Parse if(...) lines ----------
        if_match = if_pattern.match(stripped)
        if if_match:
            condition = if_match.group(1).lower()
            # If condition has 'build_testing', assume test block
            if "build_testing" in condition:
                if_stack.append("BUILD_TESTING")
                in_test_block = True
            else:
                if_stack.append("OTHER")
            continue

        # ---------- Parse endif lines ----------
        if endif_pattern.match(stripped):
            if if_stack:
                if_stack.pop()
            # Recompute in_test_block
            in_test_block = any(item == "BUILD_TESTING" for item in if_stack)
            continue

        # ---------- Parse find_package(...) lines ----------
        fp_match = find_package_pattern.match(stripped)
        if fp_match:
            inside = fp_match.group(1)  # content inside parentheses
            # e.g. 'pluginlib REQUIRED' or '${Dependency} REQUIRED'
            # Expand tokens
            expanded_tokens = inside.split()
            # remove all tokens after "COMPONENTS" (if present)
            if "COMPONENTS" in expanded_tokens:
                idx = expanded_tokens.index("COMPONENTS")
                expanded_tokens = expanded_tokens[:idx]
            # remove version constraints -> remove only number tokens e.g 3.3 or 2.0.1 or 2
            expanded_tokens = [
                tok for tok in expanded_tokens if not re.match(r"^\d+(\.\d+)*$", tok)
            ]
            # We collect all tokens that don't match known keywords like "REQUIRED", "QUIET", etc.
            skip_words = {"REQUIRED", "QUIET", "NO_MODULE"}
            used_deps = [
                tok for tok in expanded_tokens if tok.upper() not in skip_words
            ]

            add_deps(used_deps, in_test_block)
            continue

    return main_deps, test_deps


def read_cmake_file(file_path: Path) -> List[str]:
    """Reads a CMake file and returns a list of lines."""
    if isinstance(file_path, str):
        file_path = Path(file_path)
    if not file_path.exists():
        print(f"File not found: {file_path}")
        return []
    with open(file_path, "r") as f:
        raw_lines = f.readlines()
    lines = remove_comments(raw_lines)
    lines = read_cmake_lines_with_parens_joined(lines)
    lines = resolve_for_each(lines)
    # for line in lines:
    #    print(line)
    return lines


def read_deps_from_cmake_file(file_path: Path | str) -> tuple[List[str], List[str]]:
    """Reads a CMake file and returns a list of dependencies."""
    if isinstance(file_path, str):
        file_path = Path(file_path)
    lines = read_cmake_file(file_path)
    try:
        main_deps, test_deps = retrieve_cmake_dependencies(lines)
    except Exception as e:
        print(f"Error processing file {file_path}: {e}")
        return [], []
    return main_deps, test_deps


if __name__ == "__main__":
    # Example usage
    cmake_file = Path(
        "/home/aljoscha-schmidt/hector/src/hector_gamepad_manager/hector_gamepad_manager/CMakeLists.txt"
    )
    cmake_file = Path(
        "/home/aljoscha-schmidt/hector/src/hector_base_velocity_manager/hector_base_velocity_manager/CMakeLists.txt"
    )
    lines = read_cmake_file(cmake_file)
    main_deps, test_deps = retrieve_cmake_dependencies(lines)
    print("Main dependencies:", main_deps)
    print("Test dependencies:", test_deps)
