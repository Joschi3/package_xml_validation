# ROS 2 Package XML Validator & Formatter

![CI](https://github.com/Joschi3/package_xml_validation/actions/workflows/unittests.yml/badge.svg)
![Lint](https://github.com/Joschi3/package_xml_validation/actions/workflows/lint.yml/badge.svg)
[![codecov](https://codecov.io/gh/Joschi3/package_xml_validation/branch/main/graph/badge.svg)](https://codecov.io/gh/Joschi3/package_xml_validation/)

Validates and formats `package.xml` files to enforce consistency and ROS 2 schema compliance.

### ✅ What it does:

- **XML Schema Validation & Correction**
  - Validates against [package_format3.xsd](http://download.ros.org/schema/package_format3.xsd)
  - Automatically fixes ordering and formatting errors
- **Dependency Grouping & Sorting**
  - Groups dependencies by type (e.g., `build_depend`, `test_depend`)
  - Sorts alphabetically within each group
- **Non-Destructive Edits**
  - Leaves comments and indentation **unchanged**
- **Launch File Dependency Validation**
  - Scans Python (`.py`), YAML (`.yaml/.yml`), and XML (`.xml`) launch files for package references
  - Validates and corrects that all referenced packages are declared in `package.xml` (as `<exec_depend>` or `<depend>`)
  - Parses the `test` folder to extract missing `<test_depend>` dependencies
- **Rosdep Key Checking**
  - Verifies that all declared packages exist as valid rosdep keys (optional)
- **CMakeLists.txt Synchronization**
  - Compares build and test dependencies against `CMakeLists.txt` (optional)
  - Automatically inserts missing dependencies from CMake into `package.xml` as `<depend>` or `<build_depend>` (optional)
- **Export Build Type Validation**
  - Ensures `package.xml` includes the appropriate `build_type` export (e.g., `ament_cmake`, `ament_python`)
  - Validates `buildtool_depend`

#### Example: Enforced Grouping of Dependencies
```xml
<package format="3">
  ...
  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>controller_manager_msgs</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  ...
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 🛠️ Usage Example

```bash
usage: package-xml-validator [-h] [--check-only] [--file FILE] [--verbose] [--skip-rosdep-key-validation] [--compare-with-cmake] [--auto-fill-missing-deps] [--missing-deps-only] [--ignore-formatting-errors]
                             [--strict-cmake-checking]
                             [src ...]

Validate and format ROS2 package.xml files.

positional arguments:
  src                   List of files or directories to process.

options:
  -h, --help            show this help message and exit
  --check-only          Only check for errors without correcting.
  --file FILE           Path to a single XML file to process. If provided, 'src' arguments are ignored.
  --verbose             Enable verbose output.
  --skip-rosdep-key-validation
                        Check if rosdeps are valid.
  --compare-with-cmake  Check if all CMake dependencies are in package.xml.
  --auto-fill-missing-deps
                        Automatically fill missing dependencies in package.xml [--compare-with-cmake must be set].
  --missing-deps-only   Only report missing dependencies (implies --check-only).
  --ignore-formatting-errors
                        Ignore formatting-only checks (implies --check-only).
  --strict-cmake-checking
                        Treat unresolved CMake dependencies as errors instead of warnings.


```

**Example with verbose logging:**

```text
package-xml-validator ~/hector/src/hector_gamepad_manager/hector_gamepad_plugin_interface --check-only --compare-with-cmake --verbose
Processing hector_gamepad_plugin_interface...
✅ [1/13] Check for invalid tags passed.
✅ [2/13] Check for empty lines passed.
✅ [3/13] Check for duplicate elements passed.
✅ [4/13] Check element occurrences passed.
✅ [5/13] Check element order passed.
✅ [6/13] Check dependency order passed.
✅ [7/13] Check indentation passed.
✅ [8/13] Check launch dependencies passed.
✅ [9/13] Check build tool depend passed.
✅ [10/13] Check member of group passed.
✅ [11/13] Check build type export passed.
✅ [12/13] Check ROS dependencies passed.
✅ [13/13] Check CMake dependencies passed.
🎉 All `package.xml` files are valid and nicely formatted. 🚀
```

---

## ✅ Pre-commit Hook Setup

Use [`pre-commit`](https://pre-commit.com/) to automatically validate and format `package.xml` files before every commit.

### 1. Install `pre-commit`

```bash
pip install pre-commit
```

### 2. Add to `.pre-commit-config.yaml`

```yaml
repos:
  - repo: git@github.com:Joschi3/package_xml_validation.git
    rev: v1.3.0
    hooks:
      - id: format-package-xml
        name: Format package.xml
```

### 3. Install the hook (run once)

```bash
pre-commit install
```

### 4. Run manually (e.g., on first setup)

```bash
pre-commit run --all-files
```

---

## 🧪 CI Use: Check-only Mode

If you are running in CI and want to **fail on violations without modifying files**, use:

```bash
package-xml-validator --check-only --compare-with-cmake .
```

This will:

* Validate all `package.xml` files.
* Print formatting/schema issues.
* Check validity of rosdep keys.
* Compare dependencies against `CMakeLists.txt`.
* Exit with a non-zero code if problems are found.
* **Modify no files.**

*Note: If `rosdep` is not initialized in your CI environment, add `--skip-rosdep-key-validation`.*

### 🎯 Focused Modes

* `--missing-deps-only`: Skips formatting checks; only reports missing dependencies (useful for quick audits).
* `--ignore-formatting-errors`: Checks for logic/dependency errors but ignores indentation or sorting issues.

---

## ✨ Autocompletion

To enable tab autocompletion for arguments and flags:

1. **Install the package:**
```bash
pip install .
```


2. **Enable temporarily:**
Run this in your terminal:
```bash
eval "$(register-python-argcomplete package-xml-validator)"
```


3. **Enable permanently:**
Add the activation command to your shell profile (`~/.bashrc` or `~/.zshrc`):
```bash
echo 'eval "$(register-python-argcomplete package-xml-validator)"' >> ~/.bashrc
```
