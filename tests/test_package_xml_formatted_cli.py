# tests/test_package_xml_formatted_cli.py
import io
import os
import sys
import unittest
import tempfile
from pathlib import Path
from contextlib import redirect_stdout
from unittest import mock

# SUT
from package_xml_validation import package_xml_validator as SUT


def make_fake_validator_factory(
    *,
    return_from_check_and_format=True,
    return_from_check_and_format_files=True,
    constructed_out=None,
):
    """
    Build a fake PackageXmlValidator class to patch into SUT.
    Captures constructor kwargs and method calls, and returns the values provided.
    """
    constructed_out = constructed_out if constructed_out is not None else []

    class FakeValidator:
        def __init__(self, **kwargs):
            self.init_kwargs = kwargs
            self.check_and_format_calls = []
            self.check_and_format_files_calls = []
            self._ret_check_and_format = return_from_check_and_format
            self._ret_check_and_format_files = return_from_check_and_format_files
            constructed_out.append(self)

        def check_and_format(self, src):
            self.check_and_format_calls.append(list(src))
            return self._ret_check_and_format

        def check_and_format_files(self, files):
            self.check_and_format_files_calls.append(list(files))
            return self._ret_check_and_format_files

    return FakeValidator, constructed_out


class TestPackageXmlFormattedCLI(unittest.TestCase):
    def setUp(self):
        # Make a temp directory to act as predictable CWD or input paths
        self.tmp = tempfile.TemporaryDirectory()
        self.tmpdir = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_cli_defaults_uses_cwd_and_skips_rosdep_when_env_missing(self):
        """No args -> src defaults to CWD; without ROS_DISTRO, rosdep check is skipped."""
        fake_cls, constructed = make_fake_validator_factory()
        with mock.patch.object(SUT, "PackageXmlValidator", fake_cls):
            with mock.patch.object(sys, "argv", ["prog"]):
                with mock.patch.dict(os.environ, {}, clear=True):
                    with mock.patch.object(
                        SUT.os, "getcwd", return_value=str(self.tmpdir)
                    ):
                        buf = io.StringIO()
                        with redirect_stdout(buf):
                            SUT.main()
                        out = buf.getvalue()

        # One instance constructed
        self.assertEqual(len(constructed), 1)
        inst = constructed[0]

        # src defaults to CWD
        self.assertEqual(inst.check_and_format_calls, [[str(self.tmpdir)]])

        # With no ROS_DISTRO -> skip rosdep validation
        self.assertIs(inst.init_kwargs["check_rosdeps"], False)
        self.assertIn(
            "ROS_DISTRO environment variable not set. Skipping rosdep key validation.",
            out,
        )

        # When rosdep is skipped, compare_with_cmake should still be False by default
        self.assertIs(inst.init_kwargs["compare_with_cmake"], False)

    def test_cli_file_option_calls_check_and_format_files(self):
        """--file takes precedence over src; passes exact file to check_and_format_files."""
        xml = self.tmpdir / "pkg" / "package.xml"
        xml.parent.mkdir(parents=True, exist_ok=True)
        xml.write_text("<package><name>demo</name></package>", encoding="utf-8")

        fake_cls, constructed = make_fake_validator_factory()
        with mock.patch.object(SUT, "PackageXmlValidator", fake_cls):
            with mock.patch.object(sys, "argv", ["prog", "--file", str(xml)]):
                # Provide ROS_DISTRO so rosdep check is enabled
                with mock.patch.dict(os.environ, {"ROS_DISTRO": "jazzy"}, clear=True):
                    buf = io.StringIO()
                    with redirect_stdout(buf):
                        SUT.main()

        self.assertEqual(len(constructed), 1)
        inst = constructed[0]

        # check_and_format_files called with exactly the file passed
        self.assertEqual(inst.check_and_format_files_calls, [[str(xml)]])
        # path forwarded as the file
        self.assertEqual(inst.init_kwargs["path"], str(xml))
        # rosdep validation enabled when ROS_DISTRO present
        self.assertTrue(inst.init_kwargs["check_rosdeps"])

    def test_cli_compare_with_cmake_disallowed_when_skip_rosdep(self):
        """--compare-with-cmake is disabled if rosdep validation is skipped (no ROS_DISTRO)."""
        fake_cls, constructed = make_fake_validator_factory()
        with mock.patch.object(SUT, "PackageXmlValidator", fake_cls):
            with mock.patch.object(sys, "argv", ["prog", "--compare-with-cmake"]):
                with mock.patch.dict(os.environ, {}, clear=True):
                    buf = io.StringIO()
                    with redirect_stdout(buf):
                        SUT.main()
                    out = buf.getvalue()

        self.assertEqual(len(constructed), 1)
        inst = constructed[0]

        # Must print the incompatibility message and disable the flag
        self.assertIn(
            "Cannot use --compare-with-cmake with --skip-rosdep-key-validation.", out
        )
        self.assertFalse(inst.init_kwargs["check_rosdeps"])
        self.assertFalse(inst.init_kwargs["compare_with_cmake"])

    def test_cli_verbose_and_xmllint_flags_propagate(self):
        """--verbose and --check-with-xmllint should propagate to validator args."""
        fake_cls, constructed = make_fake_validator_factory()
        with mock.patch.object(SUT, "PackageXmlValidator", fake_cls):
            with mock.patch.object(
                sys, "argv", ["prog", "--verbose", "--check-with-xmllint"]
            ):
                with mock.patch.dict(os.environ, {"ROS_DISTRO": "jazzy"}, clear=True):
                    buf = io.StringIO()
                    with redirect_stdout(buf):
                        SUT.main()

        self.assertEqual(len(constructed), 1)
        inst = constructed[0]
        self.assertTrue(inst.init_kwargs["verbose"])
        self.assertTrue(inst.init_kwargs["check_with_xmllint"])
        self.assertTrue(inst.init_kwargs["check_rosdeps"])

    def test_cli_exit_code_on_failure(self):
        """When validator returns False, CLI should exit(1)."""
        # Fake returns False on both code paths
        fake_cls, constructed = make_fake_validator_factory(
            return_from_check_and_format=False,
            return_from_check_and_format_files=False,
        )
        with mock.patch.object(SUT, "PackageXmlValidator", fake_cls):
            # Use --file to take the check_and_format_files path
            xml = self.tmpdir / "p" / "package.xml"
            xml.parent.mkdir(parents=True, exist_ok=True)
            xml.write_text("<package><name>x</name></package>", encoding="utf-8")

            with mock.patch.object(sys, "argv", ["prog", "--file", str(xml)]):
                with mock.patch.dict(os.environ, {"ROS_DISTRO": "jazzy"}, clear=True):
                    with self.assertRaises(SystemExit) as cm:
                        SUT.main()
        self.assertEqual(cm.exception.code, 1)
        self.assertEqual(len(constructed), 1)
        # Sanity: we took the --file path
        self.assertEqual(constructed[0].check_and_format_files_calls, [[str(xml)]])


if __name__ == "__main__":
    unittest.main()
