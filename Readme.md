# ROS2 Package Validator

Validates and formats `package.xml` files to enforce consistency and ROS 2 schema compliance.

### ✅ What it does:
- Validates against [package_format3.xsd](http://download.ros.org/schema/package_format3.xsd) using `xmllint`
- Ensures dependencies are:
  - Grouped by type (e.g. `build_depend`, `test_depend`)
  - Sorted alphabetically within each group
- Leaves comments and indentation **unchanged**
- Removes **duplicate empty lines** between dependency groups

#### Example:
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

## ✅ Pre-commit Hook Setup

Use [`pre-commit`](https://pre-commit.com/) to automatically validate and format `package.xml` files before each commit.

### 1. Install `pre-commit`

```bash
pip install pre-commit
```

### 2. Add to `.pre-commit-config.yaml`

```yaml
repos:
  - repo: git@github.com:Joschi3/ros_pkg_validator.git
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
package-xml-formatter --check .
```

This will:
- Validate all `package.xml` files
- Print any formatting/schema issues
- Exit non-zero if any problems are found  
→ **No files will be modified**


