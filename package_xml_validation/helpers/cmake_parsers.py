from pathlib import Path
import re
from typing import Union

# -----------------------------------------------------------------------------
# Constants & Configuration
# -----------------------------------------------------------------------------

# Known CMake keys that do not need package xml <depend> entries
# These are typically system libraries or build tools provided by the environment.
CMAKE_KEYS_NO_ROSDEP = {
    "Threads",
    "OpenMP",
    "ament_cmake",  # build_tool
}

# Tokens that mark the end of the package name definition in find_package().
# Everything after these tokens (including the token itself) is part of the
# package configuration (components, paths, etc.) and not the package name.
STOP_TOKENS = {
    "COMPONENTS",
    "OPTIONAL_COMPONENTS",
    "NAMES",
    "CONFIGS",
    "HINTS",
    "PATHS",
    "PATH_SUFFIXES",
}

# Keywords that appear in the package name section but are NOT package names.
# These should be ignored when extracting the dependency.
SKIP_WORDS = {
    # Modes & Status
    "REQUIRED",
    "QUIET",
    "NO_MODULE",
    "CONFIG",
    "MODULE",
    "EXACT",
    "GLOBAL",
    "NO_POLICY_SCOPE",
    "BYPASS_PROVIDER",
    "SYSTEM",  # rare but valid in some contexts
    # Path Controls (CMake 3.24+)
    "NO_DEFAULT_PATH",
    "NO_PACKAGE_ROOT_PATH",
    "NO_CMAKE_PATH",
    "NO_CMAKE_ENVIRONMENT_PATH",
    "NO_SYSTEM_ENVIRONMENT_PATH",
    "NO_CMAKE_PACKAGE_REGISTRY",
    "NO_CMAKE_BUILDS_PATH",
    "NO_CMAKE_SYSTEM_PATH",
    "NO_CMAKE_SYSTEM_PACKAGE_REGISTRY",
    "CMAKE_FIND_ROOT_PATH_BOTH",
    "ONLY_CMAKE_FIND_ROOT_PATH",
    "NO_CMAKE_FIND_ROOT_PATH",
}


# -----------------------------------------------------------------------------
# Parsing Functions
# -----------------------------------------------------------------------------


def remove_comments(lines: list[str]) -> list[str]:
    """Remove trailing CMake comments from each line.

    Args:
        lines: Raw lines from a CMake file.

    Returns:
        A list of lines with comments removed and whitespace stripped.
    """
    return [line.split("#", 1)[0].strip() for line in lines]


def read_cmake_lines_with_parens_joined(raw_lines: list[str]) -> list[str]:
    """
    Reads a CMake file and joins lines that have an opening '(' without a matching ')'
    until the closing ')' is found. Returns a list of logically complete lines.

    Args:
        raw_lines: Raw file lines.

    Returns:
        A list of logical CMake lines with multiline blocks joined.
    """
    lines = []
    buffer = ""

    def has_balanced_parens(s: str) -> bool:
        """Check whether parentheses are balanced in a string."""
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


def resolve_for_each(raw_lines: list[str]) -> list[str]:
    """Expand simple foreach() loops into repeated lines.

    This is a basic implementation that handles `foreach(... IN LISTS ...)`
    to ensure we capture dependencies defined inside loops.

    Args:
        raw_lines: Preprocessed CMake lines.

    Returns:
        A list of lines with foreach variables expanded.
    """
    foreach_stack = []
    lines = []
    variables = {}

    # Regex patterns
    set_pattern = re.compile(
        r"^\s*set\s*\(\s*([A-Za-z0-9_]+)\s+(.*?)\)\s*$", re.IGNORECASE
    )
    foreach_pattern = re.compile(
        r"^\s*foreach\s*\(\s*([A-Za-z_][A-Za-z0-9_]*)\s+IN\s+(LISTS|ITEMS)\s+(\$\{\s*[A-Za-z0-9_]+\s*\}|[A-Za-z0-9_]+)\s*\)\s*$",
        re.IGNORECASE,
    )
    endforeach_pattern = re.compile(r"^\s*endforeach\s*\(?.*\)?\s*$", re.IGNORECASE)

    for line in raw_lines:
        stripped = line.strip()

        # ---------- Parse set(...) lines ----------
        set_match = set_pattern.match(stripped)
        if set_match:
            var_name = set_match.group(1)
            values_str = set_match.group(2)
            # Expand simple space-separated values
            expanded_vals = values_str.split()
            variables[var_name] = expanded_vals
            lines.append(line)
            continue

        # ---------- Parse foreach(...) lines ----------
        foreach_match = foreach_pattern.match(stripped)
        if foreach_match:
            loop_var = foreach_match.group(1)
            list_var = foreach_match.group(3)
            # Clean variable wrappers like ${...}
            list_var_key = list_var.strip("${}")

            # Resolve the list of values to iterate over
            expanded_vals = variables.get(list_var_key, [])
            foreach_stack.append((loop_var, expanded_vals))
            continue

        # ---------- Parse endforeach lines ----------
        if endforeach_pattern.match(stripped):
            if foreach_stack:
                foreach_stack.pop()
            continue

        # ---------- Expand Variables inside Loop ----------
        if foreach_stack:
            loop_var, expanded_vals = foreach_stack[-1]
            # If the current line uses the loop variable, replicate the line
            if f"${{{loop_var}}}" in stripped or f"${loop_var}" in stripped:
                for val in expanded_vals:
                    # Simple string replacement for ${var}
                    expanded_line = stripped.replace(f"${{{loop_var}}}", val)
                    expanded_line = expanded_line.replace(f"${loop_var}", val)
                    lines.append(expanded_line)
            else:
                lines.append(line)
        else:
            lines.append(line)

    return lines


def retrieve_cmake_dependencies(
    lines: Union[list[str], Path],
) -> tuple[list[str], list[str]]:
    """Parse CMake lines to extract main and test dependencies.

    Identifies `find_package` calls and correctly separates package names
    from version numbers, components, and CMake flags.

    Args:
        lines: CMake lines or a Path to a CMake file.

    Returns:
        Tuple of (main_deps, test_deps) as lists of unique dependency names.
    """
    if isinstance(lines, Path):
        lines = read_cmake_file(lines)

    main_deps: list[str] = []
    test_deps: list[str] = []

    # Stack to track if we are inside a test-related block
    if_stack = []
    in_test_block = False

    # Regex patterns
    if_pattern = re.compile(r"^\s*if\s*\((.*)\)\s*$", re.IGNORECASE)
    endif_pattern = re.compile(r"^\s*endif\s*\(?.*\)?\s*$", re.IGNORECASE)
    find_package_pattern = re.compile(
        r"^\s*find_package\s*\(\s*([^)]+)\)\s*$", re.IGNORECASE
    )

    def add_deps(dep_list: list[str], is_test: bool):
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
            # If condition implies testing, mark block as test
            if "build_testing" in condition or "ament_cmake_gtest" in condition:
                if_stack.append("BUILD_TESTING")
                in_test_block = True
            else:
                if_stack.append("OTHER")
            continue

        # ---------- Parse endif lines ----------
        if endif_pattern.match(stripped):
            if if_stack:
                if_stack.pop()
            # Recompute in_test_block based on remaining stack
            in_test_block = any(item == "BUILD_TESTING" for item in if_stack)
            continue

        # ---------- Parse find_package(...) lines ----------
        fp_match = find_package_pattern.match(stripped)
        if fp_match:
            inside = fp_match.group(1)
            expanded_tokens = inside.split()

            # 1. Truncate at the first Stop Token (e.g., COMPONENTS)
            #    find_package(Pkg 1.0 REQUIRED COMPONENTS a b) -> [Pkg, 1.0, REQUIRED]
            cut_index = len(expanded_tokens)
            for token in expanded_tokens:
                if token in STOP_TOKENS:
                    idx = expanded_tokens.index(token)
                    if idx < cut_index:
                        cut_index = idx

            trimmed_tokens = expanded_tokens[:cut_index]

            # 2. Filter out Version Numbers (e.g., 1.2, 3.0.0)
            trimmed_tokens = [
                tok for tok in trimmed_tokens if not re.match(r"^\d+(\.\d+)*$", tok)
            ]

            # 3. Filter out Skip Words (CMake keywords/flags)
            used_deps = [tok for tok in trimmed_tokens if tok.upper() not in SKIP_WORDS]

            add_deps(used_deps, in_test_block)
            continue

    # Clean up results
    # 1. Remove duplicates
    # 2. Remove known ignored keys (like 'Threads')

    unique_main = sorted({dep for dep in main_deps if dep not in CMAKE_KEYS_NO_ROSDEP})
    unique_test = sorted({dep for dep in test_deps if dep not in CMAKE_KEYS_NO_ROSDEP})

    return unique_main, unique_test


def read_cmake_file(file_path: Union[Path, str]) -> list[str]:
    """Read and normalize CMake file lines for dependency parsing.

    Args:
        file_path: Path to a CMakeLists.txt file.

    Returns:
        A list of normalized CMake lines.
    """
    if isinstance(file_path, str):
        file_path = Path(file_path)

    if not file_path.exists():
        # Ideally logging should be handled by a logger, keeping print for CLI compatibility
        print(f"File not found: {file_path}")
        return []

    try:
        with open(file_path, encoding="utf-8") as f:
            raw_lines = f.readlines()
    except UnicodeDecodeError:
        print(f"Error: Could not decode {file_path}. Is it a valid text file?")
        return []

    lines = remove_comments(raw_lines)
    lines = read_cmake_lines_with_parens_joined(lines)
    lines = resolve_for_each(lines)

    return lines


def read_deps_from_cmake_file(
    file_path: Union[Path, str],
) -> tuple[list[str], list[str]]:
    """Read a CMake file and return main and test dependencies.

    Args:
        file_path: Path to a CMakeLists.txt file.

    Returns:
        Tuple of (main_deps, test_deps).
    """
    try:
        main_deps, test_deps = retrieve_cmake_dependencies(Path(file_path))
        return main_deps, test_deps
    except Exception as e:
        print(f"Error processing file {file_path}: {e}")
        return [], []
