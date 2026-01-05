import unittest
from unittest.mock import MagicMock, patch, mock_open

# Import the module under test as 'SUT' (System Under Test)
from package_xml_validation.helpers import rosdep_validator as SUT


class TestRosdepValidator(unittest.TestCase):
    def setUp(self):
        # Mocks for rosdep2 components
        self.mock_installer_context = MagicMock()
        self.mock_lookup = MagicMock()
        self.mock_view = MagicMock()
        self.mock_sources_loader = MagicMock()

        # Default OS behavior
        self.mock_installer_context.get_os_name_and_version.return_value = (
            "ubuntu",
            "jammy",
        )
        self.mock_installer_context.get_sources_cache_dir.return_value = (
            "/tmp/rosdep_cache"
        )
        self.mock_installer_context.get_os_installer_keys.return_value = ["apt"]
        self.mock_installer_context.get_default_os_installer_key.return_value = "apt"

        # Patch the heavy external dependencies
        self.patcher_context = patch(
            "rosdep2.create_default_installer_context",
            return_value=self.mock_installer_context,
        )
        self.patcher_lookup = patch(
            "rosdep2.RosdepLookup.create_from_rospkg", return_value=self.mock_lookup
        )
        self.patcher_loader = patch(
            "rosdep2.sources_list.SourcesListLoader.create_default",
            return_value=self.mock_sources_loader,
        )

        self.mock_create_context = self.patcher_context.start()
        self.mock_create_lookup = self.patcher_lookup.start()
        self.mock_create_loader = self.patcher_loader.start()

        # Setup lookup to return our mock view
        self.mock_lookup.get_rosdep_view.return_value = self.mock_view

    def tearDown(self):
        self.patcher_context.stop()
        self.patcher_lookup.stop()
        self.patcher_loader.stop()

    def test_init(self):
        """Test that initialization calls the correct rosdep2 setup functions."""
        validator = SUT.RosdepValidator()
        self.mock_create_context.assert_called_once()
        self.mock_create_lookup.assert_called_once()
        self.mock_create_loader.assert_called_once()
        self.assertEqual(validator.rosdep_os_name, "ubuntu")

    def test_load_cmake_rosdep_map(self):
        """Test loading of the YAML mapping file."""
        yaml_content = "Eigen3: eigen\nBoost: boost"

        # Mock resources.files behavior
        with patch("importlib.resources.files") as mock_files:
            mock_path = MagicMock()
            mock_files.return_value.joinpath.return_value = mock_path
            # Mock opening the file
            mock_path.open = mock_open(read_data=yaml_content)

            validator = SUT.RosdepValidator()

            # Check content
            expected = {"eigen3": "eigen", "boost": "boost"}
            self.assertEqual(validator.cmake_rosdep_map, expected)

    def test_can_resolve_true(self):
        """Test a successfully resolved dependency."""
        validator = SUT.RosdepValidator()

        # Setup mock dependency object
        mock_dep_def = MagicMock()
        mock_dep_def.get_rule_for_platform.return_value = (
            "apt",
            {"package": "ros-std-msgs"},
        )
        self.mock_view.lookup.return_value = mock_dep_def

        result = validator.can_resolve("std_msgs")
        self.assertTrue(result)

    def test_can_resolve_false(self):
        """Test a dependency that fails to resolve (returns None installer)."""
        validator = SUT.RosdepValidator()

        mock_dep_def = MagicMock()
        # Installer is None -> resolution failed for this OS
        mock_dep_def.get_rule_for_platform.return_value = (None, None)
        self.mock_view.lookup.return_value = mock_dep_def

        result = validator.can_resolve("unknown_pkg")
        self.assertFalse(result)

    def test_can_resolve_exception(self):
        """Test handling of rosdep lookup exceptions."""
        validator = SUT.RosdepValidator()
        # Simulate Key Error (package not in db)
        self.mock_view.lookup.side_effect = KeyError("Key not found")

        result = validator.can_resolve("missing_pkg")
        self.assertFalse(result)

    def test_resolve_cmake_dependency_strategies(self):
        """Test the priority logic: Local -> Rosdep -> Map -> None."""

        # FIXED: Patch 'get_pkgs_in_wrs' on the MODULE (SUT), not the CLASS.
        with patch.object(SUT, "get_pkgs_in_wrs", return_value=["my_local_pkg"]):
            validator = SUT.RosdepValidator(pkg_path="/dummy")

            # Strategy 1: Local Package
            # Should return exactly what was passed if it's in the local list
            self.assertEqual(
                validator.resolve_cmake_dependency("my_local_pkg"), "my_local_pkg"
            )

            # Strategy 2: Rosdep Native
            # Setup can_resolve to return True for 'pcl_conversions'
            # We mock the method on the INSTANCE, not the class
            validator.can_resolve = MagicMock(
                side_effect=lambda x: x == "pcl_conversions"
            )
            self.assertEqual(
                validator.resolve_cmake_dependency("pcl_conversions"), "pcl_conversions"
            )

            # Strategy 3: Map
            # Inject a manual map
            validator.cmake_rosdep_map = {"eigen3": "eigen"}
            self.assertEqual(validator.resolve_cmake_dependency("Eigen3"), "eigen")

            # Strategy 4: Fail
            self.assertIsNone(validator.resolve_cmake_dependency("SuperUnknownLib"))

    def test_search_rosdep_candidates(self):
        """Test the native search functionality using mocked views."""
        validator = SUT.RosdepValidator()

        # 1. Setup Mock View Data
        mock_source_1 = MagicMock()
        # Mock CachedDataSource class signature (if needed by your code's isinstance check)
        from rosdep2.sources_list import CachedDataSource

        mock_source_1.__class__ = CachedDataSource

        mock_source_1.rosdep_data = {
            "python-numpy": {"ubuntu": "python3-numpy"},
            "libboost-dev": {"ubuntu": "libboost-all-dev"},
            "irrelevant-key": {"ubuntu": "irrelevant"},
        }

        # 2. Setup SourcesLoader to return this view
        self.mock_sources_loader.get_loadable_views.return_value = ["source1"]
        self.mock_sources_loader.get_source.return_value = mock_source_1

        # 3. Perform Search
        results = validator.search_rosdep_candidates("numpy")
        self.assertIn("python-numpy", results)

        results_boost = validator.search_rosdep_candidates("boost")
        self.assertIn("libboost-dev", results_boost)

        results_empty = validator.search_rosdep_candidates("nomatch")
        self.assertEqual(results_empty, [])

    def test_search_complex_payload(self):
        """Test searching where the match is deep inside a dict payload (like pip rules)."""
        validator = SUT.RosdepValidator()

        mock_source = MagicMock()
        from rosdep2.sources_list import CachedDataSource

        mock_source.__class__ = CachedDataSource

        mock_source.rosdep_data = {
            "custom_python_lib": {"ubuntu": {"pip": {"packages": ["complex-py-lib"]}}}
        }
        self.mock_sources_loader.get_loadable_views.return_value = ["source1"]
        self.mock_sources_loader.get_source.return_value = mock_source

        results = validator.search_rosdep_candidates("complex-py")
        self.assertIn("custom_python_lib", results)

    def test_check_rosdeps_batch(self):
        """Test batch checking of dependencies."""
        validator = SUT.RosdepValidator()

        # Mock can_resolve
        validator.can_resolve = MagicMock(side_effect=lambda x: x == "rclcpp")

        deps = ["rclcpp", "invalid_lib"]
        missing = validator.check_rosdeps(deps)

        self.assertEqual(missing, ["invalid_lib"])


if __name__ == "__main__":
    unittest.main()
