#!/usr/bin/env python3
import sys
import yaml
import pathlib

# Try importing rosdep2
try:
    import rosdep2
    from rosdep2.rospkg_loader import DEFAULT_VIEW_KEY
except ImportError:
    print(
        "Error: 'rosdep2' module not found. Please install python3-rosdep.",
        file=sys.stderr,
    )
    sys.exit(1)


def verify_mappings():
    """
    Reads the cmake_rosdep_map.yaml and verifies that every target rosdep key
    exists in the local rosdep database.
    """

    # 1. Locate the YAML file
    # Adjust this path if your script location changes relative to the data file
    try:
        # If running from source root
        yaml_path = pathlib.Path("package_xml_validation/data/cmake_rosdep_map.yaml")
        if not yaml_path.exists():
            # Fallback/alternative path strategies could go here
            print(f"Error: Could not find mapping file at {yaml_path}", file=sys.stderr)
            sys.exit(1)

        with open(yaml_path) as f:
            mapping = yaml.safe_load(f) or {}
    except Exception as e:
        print(f"Failed to load YAML file: {e}", file=sys.stderr)
        sys.exit(1)

    # 2. Setup Rosdep Lookup
    try:
        installer_context = rosdep2.create_default_installer_context()
        lookup = rosdep2.RosdepLookup.create_from_rospkg()
        view = lookup.get_rosdep_view(DEFAULT_VIEW_KEY)

        # We need an OS to check against (e.g., ubuntu) to see if a rule exists
        os_name, os_version = installer_context.get_os_name_and_version()
        print(f"Verifying against ROSDEP database for OS: {os_name} {os_version}")
    except Exception as e:
        print(
            f"Failed to initialize rosdep: {e}\nDid you run 'sudo rosdep init' and 'rosdep update'?",
            file=sys.stderr,
        )
        sys.exit(1)

    # 3. Validation Loop
    errors = []
    print(f"Checking {len(mapping)} mappings...")

    for cmake_key, rosdep_key in mapping.items():
        # Rosdep keys can be a single string or potentially a list (though your map uses strings)
        if not isinstance(rosdep_key, str):
            # Skip complex structures if they exist, or flag them
            continue

        # Check if the key exists in the view (this checks if it is defined anywhere)
        if rosdep_key not in view.keys():
            errors.append(
                f"❌ '{cmake_key}': Rosdep key '{rosdep_key}' NOT FOUND in database."
            )
            continue

        # Optional strict check: Check if it resolves for the CURRENT platform
        # This might be too strict if you map windows/macos specific keys,
        # but for standard ROS development (Ubuntu), this catches bad keys.
        try:
            dep = view.lookup(rosdep_key)
            rule = dep.get_rule_for_platform(
                os_name, os_version, ["apt", "pip", "source"], "apt"
            )
            # rule returns (installer_key, resolved_rule)
            if rule[0] is None:
                # Often not an error (just not supported on this specific OS version),
                # but worth a warning if you expect standard support.
                # Uncomment the next line to be strict:
                # errors.append(f"⚠️ '{cmake_key}': '{rosdep_key}' exists but has no rule for {os_name}:{os_version}")
                pass
        except Exception:
            errors.append(f"❌ '{cmake_key}': Error looking up '{rosdep_key}'")

    # 4. Reporting
    if errors:
        print("\nFound invalid mappings:")
        for e in errors:
            print(e)
        print("\nVerification FAILED.")
        sys.exit(1)
    else:
        print("\n✅ All mappings verified successfully.")
        sys.exit(0)


if __name__ == "__main__":
    verify_mappings()
