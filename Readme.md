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
Example:
```bash
package-xml-formatter --file ~/hector/src/hector_gamepad_manager/hector_gamepad_manager/package.xml --check_only --skip_rosdep_key_validation --verbose
Processing hector_gamepad_manager/package.xml...
✅ [1/6] All tags in hector_gamepad_manager/package.xml are valid.
✅ [2/6] No empty lines found in hector_gamepad_manager/package.xml.
✅ [3/6] No duplicate elements found in hector_gamepad_manager/package.xml.
✅ [4/6] Occurrences of elements in hector_gamepad_manager/package.xml are correct.
✅ [5/6] Element order in hector_gamepad_manager/package.xml is correct.
✅ [6/6] Dependency order in hector_gamepad_manager/package.xml is correct.
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
    rev: v0.1.5
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


