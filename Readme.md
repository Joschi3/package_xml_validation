# ROS2 Package Xml Validator & Formatter

Validates and formats `package.xml` files to enforce consistency and ROS 2 schema compliance.

### ✅ What it does:
- Validates against [package_format3.xsd](http://download.ros.org/schema/package_format3.xsd)
- Corrects erros such as
  - wrong order of xml elements
- Ensures dependencies are:
  - Grouped by type (e.g. `build_depend`, `test_depend`)
  - Sorted alphabetically within each group
- Leaves comments and indentation **unchanged**
- verifies that all rodsdep keys exist (optional)
- compares build dependencies and test dependencies with dependencies in the CMakeLists.txt (optional)


#### Example: Enforced Grouping of the dependencies
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
</package>
```
---

## 🛠️ Usage Example

```bash
package-xml-validator [-h] [--check-only] [--file FILE] [--verbose] [--check-with-xmllint] [--skip-rosdep-key-validation] [--compare-with-cmake] [src ...]

Validate and format ROS2 package.xml files.

positional arguments:
  src                           List of files or directories to process.

options:
  -h, --help                    show this help message and exit
  --check-only                  Only check for errors without correcting.
  --file FILE                   Path to a single XML file to process. If provided, 'src' arguments are ignored.
  --verbose                     Enable verbose output.
  --check-with-xmllint          Recheck XML schema using xmllint.
  --skip-rosdep-key-validation  Check if rosdeps are valid.
  --compare-with-cmake  Check if all CMake dependencies are in package.xml.
```
Example with verbose logging:
```
package-xml-validator ~/hector/src/hector_gamepad_manager/hector_gamepad_plugin_interface --check-only --compare-with-cmake --verbose
Processing hector_gamepad_plugin_interface...
✅ [1/8] Check for invalid tags passed.
✅ [2/8] Check for empty lines passed.
✅ [3/8] Check for duplicate elements passed.
✅ [4/8] Check element occurrences passed.
✅ [5/8] Check element order passed.
✅ [6/8] Check dependency order passed.
✅ [7/8] Check ROS dependencies passed.
✅ [8/8] Check CMake dependencies passed.
🎉 All `package.xml` files are valid and nicely formatted. 🚀
```

---

## ✅ Pre-commit Hook Setup

Use [`pre-commit`](https://pre-commit.com/) to automatically validate and format `package.xml` files before each commit.

### 1. Install `pre-commit`

```bash
pip install pre-commit
```

### 2. Add to `.pre-commit-config.yaml`

```yaml
repos:
  - repo: git@github.com:Joschi3/package_xml_validation.git
    rev: v1.0.4
    hooks:
      - id: format-package-xml
        name: Format package.xml
```

### 3. Install the hook (run once)

```bash
pre-commit install
```

This ensures the check runs every time you `git commit`.

### 4. Run manually (e.g. on first setup)

```bash
pre-commit run --all-files
```

---

## 🧪 CI Use: Check-only Mode

If you're running in CI and want to **fail on violations without modifying files**, use:

```bash
package-xml-formatter --check-only --compare-with-cmake .
```

This will:
- Validate all `package.xml` files
- Print any formatting/schema issues
- check validity of rosdep keys
- compare the depenedencies with the dependencies listed in the CMakeList.txt
- Exit non-zero if any problems are found  
→ **No files will be modified**
- if rosdep is not available in the CI environment use the `--skip-rosdep-key-validation` flag


