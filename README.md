# ROS 2 Package XML Validator & Formatter

![CI](https://github.com/Joschi3/package_xml_validation/actions/workflows/unittests.yml/badge.svg)
![Lint](https://github.com/Joschi3/package_xml_validation/actions/workflows/lint.yml/badge.svg)
[![codecov](https://codecov.io/gh/Joschi3/package_xml_validation/branch/main/graph/badge.svg)](https://codecov.io/gh/Joschi3/package_xml_validation/)


**Automate `package.xml` consistency in your ROS 2 projects.**

This tool checks your package manifests for required tags, schema-defined ordering, and missing dependencies discovered in `CMakeLists.txt` and launch files, then automatically formats the XML to standard conventions. It does **not** perform full XSD validation — for that, run `xmllint --schema package_format3.xsd` separately. **Designed primarily as a [pre-commit](https://pre-commit.com/) hook.**

---

## 🚀 Quick Start: Pre-commit Hook

The recommended way to use this tool is to integrate it into your `pre-commit` workflow. This ensures that every commit is automatically validated and formatted without manual intervention.

### 1. Add to `.pre-commit-config.yaml`

```yaml
repos:
  - repo: git@github.com:Joschi3/package_xml_validation.git
    rev: v1.4.2  # Use the latest tag
    hooks:
      - id: format-package-xml
        name: Format package.xml

```

### 2. Install the Hook

If you haven't already installed pre-commit hooks in your repository:

```bash
pip install pre-commit
pre-commit install

```

Now, `package.xml` files will be checked and formatted automatically on every `git commit`.

---

## 🔍 Visual Example

This tool enforces the standard ROS 2 element order:
`name` → `version` → `description` → `maintainer` → `license` → dependencies → `export`.

**Before (Disorganized & Missing Build Type):**
*Contains valid tags, but the order is random, grouping is missing, and the export tag is absent.*

```xml
<package format="3">
  <name>my_package</name>
  <description>My cool package</description>
  <version>0.0.0</version>
  <license>Apache-2.0</license>
  <maintainer email="me@example.com">Me</maintainer>
  <test_depend>ament_lint_auto</test_depend>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>std_msgs</depend>
  <depend>rclcpp</depend>
</package>

```

**After (Standardized, Sorted & Fixed):**
*Elements are reordered to match the schema, dependencies are grouped alphabetically, and missing dependencies detected in CMakeLists.txt or launch files are automatically added.*

```xml
<package format="3">
  <name>my_package</name>
  <version>0.0.0</version>
  <description>My cool package</description>
  <maintainer email="me@example.com">Me</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>example_from_cmake</depend> <!-- Automatically added missing dep from the CMakeLists.txt -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>test_launch_example</test_depend> <!-- Automatically added missing dep from a test launch file-->

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

```

---

## ✨ Features

### 1. XML Formatting & Standards

* **Required Tags:** Enforces the presence of required tags (`name`, `version`, `description`, `maintainer`, `license`) and rejects unknown top-level child tags. Does not perform full XSD validation (e.g. `format` attribute values, attribute requirements like `<maintainer email="…">`, or contents of `<export>`); pair with `xmllint --schema` if you need that.
* **Strict Ordering:** Reorders elements to match the official ROS 2 standard ([package_format3.xsd](http://download.ros.org/schema/package_format3.xsd)).
* **Intelligent Sorting:** Groups dependencies (e.g., `build_depend`, `exec_depend`) and sorts them alphabetically.
* **Non-Destructive:** Preserves your existing comments and indentation.

### 2. Dependency Integrity

* **Launch File Scanning:** Scans `.py`, `.yaml`, and `.xml` launch files. If a package is used in a launch file but missing from `package.xml`, it adds it as an `<exec_depend>` or `<test_depend>`. Can be disabled with `--skip-launch-dep-check` when launch scanning produces false positives or is not desired for a given package.
* **CMake Synchronization:** Compares `package.xml` against `CMakeLists.txt` to ensure build dependencies match, adding missing entries as `<depend>` or `<test_depend>`. Calls of the form `find_package(<pkg> QUIET)` (with `QUIET` and no `REQUIRED`) are treated as optional and skipped; all other forms — `find_package(<pkg>)`, `find_package(<pkg> REQUIRED)`, and `find_package(<pkg> REQUIRED QUIET)` — are enforced in `package.xml`.
* **Rosdep Validation:** Verifies that your dependency names exist as valid keys in the rosdep database.

### 3. Build Configuration

* **Export Validation:** Ensures the correct `<build_type>` (e.g., `ament_cmake`) is exported.
* **Test Dependencies:** Parses `test/` folders to ensure testing libraries are declared as `<test_depend>`.

---

## 🧭 Architecture

The validator is structured as a small pipeline. For each `package.xml`,
`PackageXmlValidator` parses the file once, runs a list of validation steps
against the in-memory tree, and writes back only if a step actually mutated.

| Module | Responsibility |
| --- | --- |
| `package_xml_validator.py` | CLI entry point and per-file orchestration (parse → run steps → optionally write). |
| `helpers/validation_steps/` | One `*Step` class per validation rule. Each docstring states the rule, inputs, and when (if ever) it mutates the tree. |
| `helpers/formatter/` | Pure structural checks (`structural_checks.py`), tree mutators (`mutations.py`), indentation/pretty-print helpers, and shared schema constants. `PackageXmlFormatter` is a thin facade. |
| `helpers/cmake_parsers.py` | Lightweight regex-based CMake parser used by `CMakeComparisonStep`. |
| `helpers/find_launch_dependencies.py` | Extracts package names referenced from launch files for `LaunchDependencyStep`. |
| `helpers/rosdep_validator.py`, `rosdep_wrapper.py` | Resolve rosdep keys + workspace packages; the wrapper is the single boundary against the untyped `rosdep2`. |
| `helpers/workspace.py` | ROS workspace layout discovery (locate the `<ws>/src` for a given path). |

To add a new validation rule, create a new file under `helpers/validation_steps/`
exporting a subclass of `ValidationStep`, then register it in
`PackageXmlValidator._build_steps`.

---

## 🛠️ Manual Usage (CLI)

If you need to run the validator manually or in a CI environment without pre-commit, you can install it via pip.

### Installation

```bash
pip install package-xml-validator
# OR install from source
pip install .

```

### Usage Examples


**Check only (Don't modify files)**

```bash
package-xml-validator . --check-only

```

**Auto-fill missing dependencies from CMake**

```bash
package-xml-validator . --compare-with-cmake --auto-fill-missing-deps

```

### CLI Options

| Option | Description |
| --- | --- |
| `--check-only` | Report errors/formatting issues without modifying files (Exit code 1 on failure). |
| `--compare-with-cmake` | Check if dependencies used in `CMakeLists.txt` are declared in `package.xml`. |
| `--auto-fill-missing-deps` | Automatically add dependencies found in CMake/Launch files to `package.xml`. |
| `--strict-cmake-checking` | Treat unresolved CMake dependencies as errors instead of warnings. |
| `--skip-rosdep-key-validation` | Skip verifying if dependency names exist in the `rosdep` database. |
| `--missing-deps-only` | Skips formatting checks; only looks for missing dependencies. |
| `--ignore-cmake-key KEY` | Treat `find_package(KEY ...)` in `CMakeLists.txt` as not requiring a `package.xml` `<depend>` entry. Repeatable. Merged with the built-in defaults (`Threads`, `OpenMP`, `ament_cmake`). |
| `--ignore-deps dep1,dep2` | Comma-separated list of dependency names to globally ignore in validation. |
| `--skip-launch-dep-check` | Skip checking for missing dependencies in launch and test files. |

---

## Ignoring Dependencies

Some dependencies detected in `CMakeLists.txt` or launch files should not be declared in `package.xml` — typically when a package pulls in heavy transitive dependencies (e.g. `rviz2` and its GUI stack) that should not be installed on every target machine.

> **Prefer splitting the package first.** For example, rather than a single `robot_description` package containing both URDF/xacro files *and* an RViz visualization launch file, split it into `robot_description` (model files) and `robot_description_visualization` (RViz launch files). Each package then declares only what it truly needs, and deploying `robot_description` to a robot won't drag in `rviz2`.

If splitting is not practical, the tool recognizes a `validator:ignore` directive inside XML comments:

```xml
<package format="3">
  <name>robot_description</name>
  <!-- ... -->

  <!-- validator:ignore rviz2 joint_state_publisher_gui -->

  <buildtool_depend>ament_cmake</buildtool_depend>
  <!-- ... -->
</package>
```

- Names are space-separated after `validator:ignore`.
- The directive may appear anywhere inside the `<package>` element; multiple directives in the same file are merged.
- Listed dependencies are neither flagged as missing nor added by `--auto-fill-missing-deps`.
- Scope is per-file — each `package.xml` manages its own ignore list.

For a global override — typically in CI — use the `--ignore-deps` CLI argument:

```bash
package-xml-validator . --compare-with-cmake --ignore-deps rviz2,joint_state_publisher_gui
```

---

## 🧪 CI Integration

To run this in GitHub Actions or GitLab CI (outside of pre-commit), use the check-only mode.

```bash
package-xml-validator --check-only --compare-with-cmake .

```

*Note: If `rosdep` is not initialized in your CI environment, add `--skip-rosdep-key-validation` to avoid errors.*

---

## ⌨️ Autocompletion

To enable tab autocompletion for CLI arguments:

1. **Install:** `pip install .`
2. **Enable (Temporary):** `eval "$(register-python-argcomplete package-xml-validator)"`
3. **Enable (Permanent):** `echo 'eval "$(register-python-argcomplete package-xml-validator)"' >> ~/.bashrc`
