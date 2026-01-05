# ROS 2 Package XML Validator & Formatter

![CI](https://github.com/Joschi3/package_xml_validation/actions/workflows/unittests.yml/badge.svg)
![Lint](https://github.com/Joschi3/package_xml_validation/actions/workflows/lint.yml/badge.svg)
[![codecov](https://codecov.io/gh/Joschi3/package_xml_validation/branch/main/graph/badge.svg)](https://codecov.io/gh/Joschi3/package_xml_validation/)


**Automate `package.xml` consistency in your ROS 2 projects.**

This tool validates your package manifests against the ROS 2 schema, checks for missing dependencies in code/launch files, and automatically formats the XML to standard conventions. **It is designed primarily to be used as a [pre-commit](https://pre-commit.com/) hook.**

---

## 🚀 Quick Start: Pre-commit Hook

The recommended way to use this tool is to integrate it into your `pre-commit` workflow. This ensures that every commit is automatically validated and formatted without manual intervention.

### 1. Add to `.pre-commit-config.yaml`

```yaml
repos:
  - repo: git@github.com:Joschi3/package_xml_validation.git
    rev: v1.3.0  # Use the latest tag
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

* **Schema Compliance:** Enforces the presence of required tags (`name`, `version`, `description`, `maintainer`, `license`).
* **Strict Ordering:** Reorders elements to match the official ROS 2 standard ([package_format3.xsd](http://download.ros.org/schema/package_format3.xsd)).
* **Intelligent Sorting:** Groups dependencies (e.g., `build_depend`, `exec_depend`) and sorts them alphabetically.
* **Non-Destructive:** Preserves your existing comments and indentation.

### 2. Dependency Integrity

* **Launch File Scanning:** Scans `.py`, `.yaml`, and `.xml` launch files. If a package is used in a launch file but missing from `package.xml`, it adds it as an `<exec_depend>` or `<test_depend>`.
* **CMake Synchronization:** Compares `package.xml` against `CMakeLists.txt` to ensure build dependencies match. It adds missing as `<depend>` or `<test_depend>`.
* **Rosdep Validation:** Verifies that your dependency names exist as valid keys in the rosdep database.

### 3. Build Configuration

* **Export Validation:** Ensures the correct `<build_type>` (e.g., `ament_cmake`) is exported.
* **Test Dependencies:** Parses `test/` folders to ensure testing libraries are declared as `<test_depend>`.

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
